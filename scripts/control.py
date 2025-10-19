import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue
import math
import re

class SerialTerminal:
    def __init__(self, root):
        self.root = root
        self.root.title("Terminal Serial Python")
        self.root.geometry("700x500")

        self.serial_port = None
        self.read_thread = None
        self.port_queue = queue.Queue()
        self.running = False

        # --- Frame de Conexión ---
        connection_frame = ttk.LabelFrame(root, text="Conexión")
        connection_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(connection_frame, text="Puerto:").pack(side=tk.LEFT, padx=5, pady=5)
        
        # Combobox para listar puertos
        self.port_combo = ttk.Combobox(connection_frame, width=20)
        self.port_combo.pack(side=tk.LEFT, padx=5, pady=5)
        
        ttk.Label(connection_frame, text="Baudrate:").pack(side=tk.LEFT, padx=5, pady=5)
        
        # Combobox para Baudrates
        self.baud_combo = ttk.Combobox(connection_frame, width=10, values=[
            "9600", "19200", "38400", "57600", "115200"
        ])
        self.baud_combo.set("115200") # Valor por defecto
        self.baud_combo.pack(side=tk.LEFT, padx=5, pady=5)

        # Botones
        self.connect_button = ttk.Button(connection_frame, text="Conectar", command=self.connect_serial)
        self.connect_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.disconnect_button = ttk.Button(connection_frame, text="Desconectar", command=self.disconnect_serial, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.refresh_button = ttk.Button(connection_frame, text="Refrescar", command=self.populate_ports)
        self.refresh_button.pack(side=tk.LEFT, padx=5, pady=5)

        # --- Frame de Display (Terminal) ---
        display_frame = ttk.LabelFrame(root, text="Consola")
        display_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Usamos ScrolledText para tener una barra de scroll automática
        self.text_display = scrolledtext.ScrolledText(display_frame, wrap=tk.WORD, state=tk.DISABLED)
        self.text_display.pack(fill="both", expand=True)

        # --- Frame del Gráfico de Posición ---
        # Gráfico circular con un vector que indica el ángulo del motor
        plot_frame = ttk.LabelFrame(root, text="Posición del Motor")
        plot_frame.pack(fill="x", padx=10, pady=5)

        self.angle_label = ttk.Label(plot_frame, text="Ángulo: 0.0°")
        self.angle_label.pack(anchor=tk.W, padx=5, pady=(5, 0))

        self.canvas_size = 240
        self.canvas = tk.Canvas(plot_frame, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.pack(padx=5, pady=5)

        # Geometría del gráfico
        self.cx = self.canvas_size // 2
        self.cy = self.canvas_size // 2
        self.radius = self.canvas_size // 2 - 12
        self.vector_id = None
        self._draw_position_dial()

        # --- Frame de Target Angle ---
        target_frame = ttk.LabelFrame(root, text="Target Angle")
        target_frame.pack(fill="x", padx=10, pady=5)

        self.target_var = tk.DoubleVar(value=0.0)
        self.target_label = ttk.Label(target_frame, text="Target: 0°")
        self.target_label.pack(anchor=tk.W, padx=5, pady=(5, 0))

        self.target_scale = ttk.Scale(
            target_frame,
            from_=-180.0,
            to=180.0,
            orient=tk.HORIZONTAL,
            variable=self.target_var,
            command=self._on_target_change,
            state=tk.DISABLED,
            length=500,
        )
        self.target_scale.pack(fill="x", padx=8, pady=8)
        # Enviar solo cuando sueltas el mouse
        self.target_scale.bind("<ButtonRelease-1>", self._on_target_released)

        # --- Frame de Envío ---
        send_frame = ttk.LabelFrame(root, text="Enviar Datos")
        send_frame.pack(fill="x", padx=10, pady=5)

        self.entry_send = ttk.Entry(send_frame)
        self.entry_send.pack(fill="x", expand=True, side=tk.LEFT, padx=5, pady=5)
        
        # Bind <Return> (Enter) a la función de enviar
        self.entry_send.bind("<Return>", self.send_data)

        self.send_button = ttk.Button(send_frame, text="Enviar", command=self.send_data, state=tk.DISABLED)
        self.send_button.pack(side=tk.LEFT, padx=5, pady=5)

        # Botón toggle Encendido/Apagado (envía comando 'S')
        self.toggle_var = tk.BooleanVar(value=False)
        self.toggle_button = ttk.Checkbutton(
            send_frame,
            text="Motor OFF",
            variable=self.toggle_var,
            command=self.toggle_motor,
            state=tk.DISABLED
        )
        self.toggle_button.pack(side=tk.LEFT, padx=8, pady=5)

        # --- Fin de la inicialización ---
        self.populate_ports()
        # Iniciar el chequeo de la cola para datos recibidos
        self.root.after(100, self.check_queue)
        # Configurar el cierre limpio de la app
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def populate_ports(self):
        """ Llena la lista de puertos serie disponibles """
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        # Intenta pre-seleccionar el puerto que pidió el usuario
        if "/dev/ttyUSB0" in ports:
            self.port_combo.set("/dev/ttyUSB0")
        elif ports:
            self.port_combo.set(ports[0])

    def connect_serial(self):
        """ Inicia la conexión serial y el hilo de lectura """
        if self.serial_port:
            self.log_to_display("¡Ya estás conectado!\n")
            return

        port = self.port_combo.get()
        baud = self.baud_combo.get()

        if not port or not baud:
            self.log_to_display("Error: Selecciona puerto y baudrate.\n")
            return

        try:
            # Abre el puerto serial con un timeout
            self.serial_port = serial.Serial(port, int(baud), timeout=1)
            
            self.running = True
            # Inicia el hilo de lectura. daemon=True hace que el hilo muera si la app principal se cierra.
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()

            # Actualiza la GUI
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.send_button.config(state=tk.NORMAL)
            self.toggle_button.config(state=tk.NORMAL)
            self.target_scale.config(state=tk.NORMAL)
            self.port_combo.config(state=tk.DISABLED)
            self.baud_combo.config(state=tk.DISABLED)

            self.log_to_display(f"Conectado a {port} @ {baud} bps\n")

        except serial.SerialException as e:
            self.log_to_display(f"Error al conectar: {e}\n")
            self.serial_port = None

    def disconnect_serial(self):
        """ Cierra la conexión serial y detiene el hilo """
        if self.serial_port:
            self.running = False # Señal para que el hilo de lectura se detenga
            
            # Esperar un poco a que el hilo termine (opcional pero buena práctica)
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=0.1)

            self.serial_port.close()
            self.serial_port = None

            # Actualiza la GUI
            self.connect_button.config(state=tk.NORMAL)
            self.disconnect_button.config(state=tk.DISABLED)
            self.send_button.config(state=tk.DISABLED)
            self.toggle_button.config(state=tk.DISABLED)
            self.target_scale.config(state=tk.DISABLED)
            self.port_combo.config(state=tk.NORMAL)
            self.baud_combo.config(state=tk.NORMAL)

            self.log_to_display("Desconectado.\n")

    def read_serial_data(self):
        """
        Función que se ejecuta en un hilo separado para leer datos.
        Lee líneas de texto (terminadas en '\n').
        """
        while self.running and self.serial_port:
            try:
                # readline() es una operación bloqueante, pero tiene timeout
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline()
                    if data:
                        # Pone los datos en la cola para que el hilo principal (GUI) los procese
                        self.port_queue.put(data)
            except serial.SerialException:
                # Si hay un error (ej: se desconecta el cable), salimos del bucle
                self.port_queue.put("¡Error de serie! Desconectando...\n".encode('utf-8'))
                self.running = False # Detiene el bucle
                # Pedimos a la GUI que se desconecte de forma segura
                self.root.after(0, self.disconnect_serial)
                break
            except Exception as e:
                print(f"Error en hilo: {e}") # Debug

    def check_queue(self):
        """ Revisa la cola de datos desde el hilo principal de la GUI """
        while not self.port_queue.empty():
            data = self.port_queue.get()
            try:
                # Decodifica los bytes a string (UTF-8 es común, ajusta si es necesario)
                message = data.decode('utf-8', errors='replace')
                self.log_to_display(message)
                # Intentar extraer y actualizar ángulo si el mensaje lo contiene
                self._parse_and_update_angle(message)
            except UnicodeDecodeError:
                self.log_to_display(f"[Datos Binarios]: {repr(data)}\n")
        
        # Vuelve a programar esta función para que se ejecute en 100ms
        self.root.after(100, self.check_queue)

    def send_data(self, event=None):
        """ Envía datos desde el campo de entrada """
        if self.serial_port and self.serial_port.is_open:
            data = self.entry_send.get()
            if data:
                # Añade un salto de línea (común en terminales) y codifica a bytes
                data_with_newline = (data + '\n').encode('utf-8')
                
                try:
                    self.serial_port.write(data_with_newline)
                    # Opcional: mostrar lo que enviaste (eco local)
                    self.log_to_display(f"TX: {data}\n")
                    # Limpia el campo de entrada
                    self.entry_send.delete(0, tk.END)
                except serial.SerialException as e:
                    self.log_to_display(f"Error al enviar: {e}\n")
        else:
            self.log_to_display("No estás conectado.\n")

    def log_to_display(self, message):
        """ Añade texto de forma segura al widget Text """
        self.text_display.config(state=tk.NORMAL)
        self.text_display.insert(tk.END, message)
        self.text_display.see(tk.END) # Auto-scroll
        self.text_display.config(state=tk.DISABLED)
        
    def on_closing(self):
        """ Maneja el cierre de la ventana """
        self.disconnect_serial() # Cierra el puerto limpiamente
        self.root.destroy() # Cierra la app

    def toggle_motor(self):
        """Envía el comando de toggle 'S' y actualiza el texto del botón."""
        # Actualiza texto según estado local
        if self.toggle_var.get():
            self.toggle_button.config(text="Motor ON")
        else:
            self.toggle_button.config(text="Motor OFF")

        # Enviar comando 'S' si está conectado
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"S\n")
                self.log_to_display("TX: S\n")
            except serial.SerialException as e:
                self.log_to_display(f"Error al enviar: {e}\n")

    # --- Target angle handlers ---
    def _on_target_change(self, value: str):
        """Actualiza la etiqueta de target al mover el slider (sin enviar)."""
        try:
            v = float(value)
        except Exception:
            v = self.target_var.get()
        v = max(-180.0, min(180.0, v))
        self.target_label.config(text=f"Target: {v:.0f}°")

    def _on_target_released(self, _event=None):
        """Envía el comando T<deg> cuando se suelta el slider."""
        deg = int(round(self.target_var.get()))
        deg = max(-180, min(180, deg))
        self._send_target_command(deg)

    def _send_target_command(self, deg: int):
        if self.serial_port and self.serial_port.is_open:
            try:
                cmd = f"T{deg}\n".encode("utf-8")
                self.serial_port.write(cmd)
                self.log_to_display(f"TX: T{deg}\n")
            except serial.SerialException as e:
                self.log_to_display(f"Error al enviar: {e}\n")
        else:
            self.log_to_display("No estás conectado.\n")

    # --- Métodos del gráfico de posición ---
    def _draw_position_dial(self):
        """Dibuja el dial circular y marcas básicas una vez."""
        # Círculo exterior
        self.canvas.create_oval(
            self.cx - self.radius,
            self.cy - self.radius,
            self.cx + self.radius,
            self.cy + self.radius,
            outline="#444", width=2
        )

        # Ejes principales (0°, 90°, 180°, 270°)
        for ang in (0, 90, 180, 270):
            x, y = self._point_on_circle(ang, self.radius)
            self.canvas.create_line(self.cx, self.cy, x, y, fill="#ddd")

        # Marcas cada 30°
        for ang in range(0, 360, 30):
            x1, y1 = self._point_on_circle(ang, self.radius - 8)
            x2, y2 = self._point_on_circle(ang, self.radius)
            self.canvas.create_line(x1, y1, x2, y2, fill="#666")

        # Vector inicial en 0°
        self._update_position_vector(0.0)

    def _point_on_circle(self, angle_deg: float, r: float):
        """Devuelve coordenadas (x,y) en el canvas para un ángulo en grados."""
        rad = math.radians(angle_deg)
        x = self.cx + r * math.cos(rad)
        y = self.cy - r * math.sin(rad)  # invertimos Y para canvas
        return x, y

    def _update_position_vector(self, angle_deg: float):
        """Actualiza el vector en el gráfico al ángulo indicado (grados)."""
        angle_norm = (angle_deg % 360 + 360) % 360
        tip_x, tip_y = self._point_on_circle(angle_norm, self.radius - 14)

        if self.vector_id is None:
            self.vector_id = self.canvas.create_line(
                self.cx, self.cy, tip_x, tip_y,
                fill="#d22", width=3, arrow=tk.LAST
            )
        else:
            self.canvas.coords(self.vector_id, self.cx, self.cy, tip_x, tip_y)

        self.angle_label.config(text=f"Ángulo: {angle_norm:.1f}°")

    def _parse_and_update_angle(self, message: str):
        """Extrae el ángulo desde el formato fijo y actualiza el gráfico.

        Formato esperado (ejemplo):
        "I (40173) MOTOR_FOC_TORQUE: Angle: -0.00 rad, Velocity: 54547.52 rad/s, Target: 0.00 rad"
        """
        line = message.strip()
        if not line:
            return

        # Buscar explícitamente el valor que sigue a "Angle:"
        m = re.search(r"Angle:\s*([+-]?\d+(?:[\.,]\d+)?)\s*(?:rad)?", line, re.IGNORECASE)
        if not m:
            return
        number_str = m.group(1).replace(",", ".")
        try:
            angle_rad = float(number_str)
        except ValueError:
            return

        angle_deg = math.degrees(angle_rad)
        self._update_position_vector(angle_deg)

# --- Bloque principal para ejecutar la aplicación ---
if __name__ == "__main__":
    import sys
    simulate = len(sys.argv) > 1 and sys.argv[1] == "--simulate"

    root = tk.Tk()
    app = SerialTerminal(root)

    if simulate:
        # Alimenta ángulos de prueba en la consola y el gráfico
        def _feed():
            # Genera un ángulo girando a 30 deg/s y emite el formato fijo de log
            if not hasattr(_feed, "t"):
                _feed.t = 0.0
                _feed.ts = 0
            _feed.t += 0.1
            _feed.ts += 100
            angle_deg = (30.0 * _feed.t) % 360.0
            angle_rad = math.radians(angle_deg)
            vel_rads = 2.0  # valor ficticio
            target_rad = 0.0
            msg = (
                f"I ({_feed.ts}) MOTOR_FOC_TORQUE: Angle: {angle_rad:.2f} rad, "
                f"Velocity: {vel_rads:.2f} rad/s, Target: {target_rad:.2f} rad\n"
            )
            app.port_queue.put(msg.encode("utf-8"))
            root.after(100, _feed)

        root.after(200, _feed)

    root.mainloop()