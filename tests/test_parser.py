import pytest

from serial_plot import parse_line


def test_parse_valid_line():
    line = "I (99134) MOTOR_FOC_TORQUE: Angle: 3.14 rad, Velocity: 0.69 rad/s"
    ts, angle, vel = parse_line(line)
    assert ts == 99134
    assert angle == pytest.approx(3.14)
    assert vel == pytest.approx(0.69)


@pytest.mark.parametrize(
    "line",
    [
        "I (123) TAG: Angle: -1.0, Velocity: 2.5",
        "I (0) TAG: Velocity: 0.0 rad/s, Angle: 1.57 rad",  # swapped order won't match fully
        "No match here",
    ],
)
def test_parse_invalid_or_partial(line):
    ts, angle, vel = parse_line(line)
    # ts may be present or not; we check angle/vel parsing result
    assert angle is None or vel is None
