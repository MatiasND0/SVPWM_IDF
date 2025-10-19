#!/usr/bin/env python3
"""
Serial live plotter for ESP-IDF logs.

Parses lines like:
  I (99134) MOTOR_FOC_TORQUE: Angle: 3.14 rad, Velocity: 0.69 rad/s

and plots Angle [rad] and Velocity [rad/s] in real time.

Usage:
  python3 serial_plot.py --port /dev/ttyUSB1 --baud 115200

Dependencies: pyserial, matplotlib
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple
import math
from queue import Queue, Empty

try:
    import serial  # type: ignore
except Exception as e:  # pragma: no cover - handled at runtime
    serial = None  # type: ignore


# Regex patterns to extract timestamp (ms), angle (rad), velocity (rad/s)
TS_RE = re.compile(r"\bI\s*\((?P<ts>\d+)\)")
AV_RE = re.compile(
    r"Angle:\s*(?P<angle>[+-]?\d+(?:\.\d+)?)(?:\s*rad)?\s*,\s*Velocity:\s*(?P<vel>[+-]?\d+(?:\.\d+)?)(?:\s*rad/s)?",
    re.IGNORECASE,
)


@dataclass
class Sample:
    t_s: float  # seconds since start (monotonic)
    angle: float
    velocity: float


def parse_line(line: str) -> Tuple[Optional[int], Optional[float], Optional[float]]:
    """Parse a log line and return (timestamp_ms, angle, velocity).

    Returns (None, None, None) if not matched.
    """
    if not line:
        return None, None, None

    ts_ms: Optional[int] = None
    ts_match = TS_RE.search(line)
    if ts_match:
        try:
            ts_ms = int(ts_match.group("ts"))
        except ValueError:
            ts_ms = None

    av_match = AV_RE.search(line)
    if not av_match:
        return ts_ms, None, None

    try:
        angle = float(av_match.group("angle"))
        vel = float(av_match.group("vel"))
    except ValueError:
        return ts_ms, None, None

    return ts_ms, angle, vel


def _reader_thread(
    port: str,
    baud: int,
    stop_event: threading.Event,
    queue: Deque[Sample],
    cmd_queue: Optional[Queue[str]] = None,
):
    if serial is None:
        print("pyserial is not installed. Please install with: pip install pyserial", file=sys.stderr)
        stop_event.set()
        return

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"Error opening serial port {port} @ {baud}: {e}", file=sys.stderr)
        stop_event.set()
        return

    start_time = time.monotonic()
    try:
        while not stop_event.is_set():
            # handle outgoing commands first (non-blocking)
            if cmd_queue is not None:
                for _ in range(10):  # drain up to 10 cmds per loop to avoid starving reads
                    try:
                        cmd = cmd_queue.get_nowait()
                    except Empty:
                        break
                    try:
                        if not cmd.endswith("\n"):
                            cmd += "\n"
                        ser.write(cmd.encode("utf-8"))
                        ser.flush()
                    except Exception:
                        # ignore write errors; continue
                        pass

            try:
                raw = ser.readline()
            except Exception:
                # transient read error; continue
                continue
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            ts_ms, angle, vel = parse_line(line)
            if angle is None or vel is None:
                continue
            t_s = time.monotonic() - start_time
            queue.append(Sample(t_s=t_s, angle=angle, velocity=vel))
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Live plot Angle and Velocity from ESP-IDF serial logs")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB1", help="Serial port (e.g., /dev/ttyUSB1)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--max-points", type=int, default=1000, help="Rolling window length (samples)")
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI; print parsed values")
    parser.add_argument("--show-ts", action="store_true", help="Show timestamp in prints when --no-gui is used")
    args = parser.parse_args(argv)

    # Prepare data queue
    q: Deque[Sample] = deque(maxlen=max(10, args.max_points))
    cmd_q: Queue[str] = Queue()
    stop_event = threading.Event()

    # Start reader thread
    t = threading.Thread(target=_reader_thread, args=(args.port, args.baud, stop_event, q, cmd_q), daemon=True)
    t.start()

    if args.no_gui:
        try:
            while not stop_event.is_set():
                if q:
                    s = q[-1]
                    if args.show_ts:
                        print(f"t={s.t_s:7.3f}s angle={s.angle: .4f} rad vel={s.velocity: .4f} rad/s")
                    else:
                        print(f"angle={s.angle: .4f} rad vel={s.velocity: .4f} rad/s")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            t.join(timeout=2)
        return 0

    # GUI path
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation  # type: ignore
        from matplotlib.widgets import Slider, Button  # type: ignore
    except Exception:
        print("matplotlib is not installed. Please install with: pip install matplotlib", file=sys.stderr)
        stop_event.set()
        t.join(timeout=2)
        return 2

    plt.style.use("seaborn-v0_8-darkgrid")
    from matplotlib import gridspec  # type: ignore
    fig = plt.figure(figsize=(11, 7))
    fig.canvas.manager.set_window_title("Serial Plot: Angle/Velocity (with Polar Position)")
    gs = gridspec.GridSpec(2, 2, height_ratios=[1, 1.1], width_ratios=[1.2, 1], hspace=0.3, wspace=0.25)
    ax1 = fig.add_subplot(gs[0, 0])  # Angle vs Time
    axp = fig.add_subplot(gs[0, 1], projection="polar")  # Polar position
    ax2 = fig.add_subplot(gs[1, :], sharex=ax1)  # Velocity vs Time
    # Leave space at bottom for controls
    plt.subplots_adjust(bottom=0.18)

    xdata: list[float] = []
    y_angle: list[float] = []
    y_vel: list[float] = []

    line_angle, = ax1.plot([], [], label="Angle [rad]", color="tab:blue")
    line_vel, = ax2.plot([], [], label="Velocity [rad/s]", color="tab:orange")
    # Polar vector artists (angle as theta, radius fixed to 1)
    line_vec, = axp.plot([], [], color="tab:green", linewidth=2)
    point_vec, = axp.plot([], [], color="tab:green", marker="o")

    ax1.set_ylabel("Angle [rad]")
    ax2.set_ylabel("Velocity [rad/s]")
    ax2.set_xlabel("Time [s]")
    ax1.legend(loc="upper right")
    ax2.legend(loc="upper right")
    # Polar axis cosmetics
    axp.set_title("Position (polar)")
    axp.set_theta_zero_location("E")  # 0 rad points to the right
    axp.set_theta_direction(1)  # counter-clockwise positive
    axp.set_thetalim(0, 2 * math.pi)
    axp.set_rlim(0, 1.0)
    axp.set_rticks([1.0])
    axp.grid(True)

    # Controls: slider (degrees) and send button
    slider_ax = fig.add_axes([0.12, 0.05, 0.55, 0.04])
    btn_ax = fig.add_axes([0.72, 0.045, 0.15, 0.05])
    s_angle = Slider(slider_ax, label="Target Angle [deg]", valmin=-180.0, valmax=180.0, valinit=0.0)
    btn_send = Button(btn_ax, label="Send (T<deg>)", hovercolor="0.9")

    def _send_current_angle(_event=None):
        deg = float(s_angle.val)
        cmd = f"T{deg:.2f}"
        try:
            cmd_q.put(cmd, block=False)
        except Exception:
            pass

    btn_send.on_clicked(_send_current_angle)

    def init():
        ax1.set_xlim(0, 10)
        ax2.set_xlim(0, 10)
        ax1.set_ylim(-3.5, 3.5)
        ax2.set_ylim(-20, 20)
        # initialize polar vector pointing to 0
        line_vec.set_data([0, 0], [0, 1])
        point_vec.set_data([0], [1])
        return line_angle, line_vel, line_vec, point_vec

    def update(_frame):
        # drain queue
        while q:
            s = q.popleft()
            xdata.append(s.t_s)
            y_angle.append(s.angle)
            y_vel.append(s.velocity)
            # trim to max points
            if len(xdata) > args.max_points:
                xdata[:] = xdata[-args.max_points :]
                y_angle[:] = y_angle[-args.max_points :]
                y_vel[:] = y_vel[-args.max_points :]

        if not xdata:
            return line_angle, line_vel, line_vec, point_vec

        # adaptive xlim
        xmin = max(0.0, xdata[-1] - 10.0)
        xmax = xdata[-1] if xdata[-1] > 10 else 10
        ax1.set_xlim(xmin, xmax)
        ax2.set_xlim(xmin, xmax)

        line_angle.set_data(xdata, y_angle)
        line_vel.set_data(xdata, y_vel)
        # update polar vector with latest angle (wrap to [0, 2π))
        theta = y_angle[-1]
        if theta is not None:
            theta = math.fmod(theta, 2 * math.pi)
            if theta < 0:
                theta += 2 * math.pi
            line_vec.set_data([theta, theta], [0, 1])
            point_vec.set_data([theta], [1])
        # optional: autoscale y based on recent data
        for ax, y in ((ax1, y_angle), (ax2, y_vel)):
            if len(y) >= 10:
                ywin = y[-min(200, len(y)) :]
                ymin = min(ywin)
                ymax = max(ywin)
                if ymin == ymax:
                    ymin -= 1
                    ymax += 1
                pad = 0.1 * (ymax - ymin)
                ax.set_ylim(ymin - pad, ymax + pad)
        return line_angle, line_vel, line_vec, point_vec

    ani = FuncAnimation(fig, update, init_func=init, interval=1, blit=False)

    try:
        # keep some room for bottom widgets
        plt.tight_layout(rect=[0, 0.08, 1, 1])
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        t.join(timeout=2)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
