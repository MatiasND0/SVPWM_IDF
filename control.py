import serial
import time

# Configuración del puerto serial
puerto = '/dev/ttyUSB0'
baudrate = 115200  # Ajusta según tu dispositivo
timeout = 1

try:
    # Abrir el puerto serial
    ser = serial.Serial(puerto, baudrate, timeout=timeout)
    print(f"Puerto {puerto} abierto correctamente")
    
    # Esperar a que el puerto esté listo
    time.sleep(2)
    
    # Leer datos del puerto
    while True:
        if ser.in_waiting > 0:
            datos = ser.readline().decode('utf-8').rstrip()
            print(f"Datos recibidos: {datos}")
        
        time.sleep(0.1)

except serial.SerialException as e:
    print(f"Error al abrir el puerto: {e}")
except KeyboardInterrupt:
    print("\nPrograma interrumpido por el usuario")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Puerto cerrado")