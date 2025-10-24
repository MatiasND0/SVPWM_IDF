# SVPWM_IDF — Control FOC de motor BLDC con ESP-IDF + SimpleFOC

Firmware para ESP32 que implementa control FOC (Field Oriented Control) de posición sobre un motor BLDC usando 6PWM (SVPWM por defecto), con sensor magnético AS5600 por I2C y consola UART para ajustar parámetros en tiempo real.

El proyecto usa FreeRTOS (tareas para control y consola), la librería esp-simplefoc como componente administrado, y está listo para compilarse con ESP-IDF.


## Qué hace este firmware

- Inicializa un driver 6PWM basado en MCPWM y un sensor AS5600 (I2C) para estimar ángulo del eje.
- Ejecuta FOC en modo control de posición (MotionControlType::angle), con controlador de torque por voltaje.
- Utiliza modulación Space Vector PWM (SVPWM) por defecto, con posibilidad de cambiar a SinePWM o trapezoidal desde consola.
- Ofrece una consola por UART0 (115200-8-N-1) para:
	- Fijar target de posición en grados.
	- Ajustar PID de velocidad (P/I/D) en caliente.
	- Cambiar tipo de modulación FOC.
	- Habilitar/Deshabilitar el motor y consultar estado.


## Hardware y pines

Parámetros principales definidos en `main/main.cpp`:

- Pares de polos del motor: `POLE_PAIRS = 7`
- Pines 6PWM (alto/bajo por fase):
	- U: UH = 25, UL = 26
	- V: VH = 27, VL = 14
	- W: WH = 32, WL = 33
- I2C (AS5600): SDA = 21, SCL = 22

Notas:
- Asegúrate de que tu etapa de potencia (puente trifásico/MOSFET driver) sea compatible con 6 señales PWM complementarias.
- Ajusta `driver.voltage_power_supply` y límites de voltaje según tu fuente/etapa de potencia.
- Verifica alimentación y masa común entre ESP32, driver y sensor.


## Dependencias

- ESP-IDF instalado y configurado (5.x o superior recomendado).
- Componentes administrados (ya referenciados en `main/idf_component.yml`), incluyendo `esp_simplefoc` y dependencias.
- Para scripts opcionales en `scripts/`, instala Python 3.8+ y los paquetes de `scripts/requirements.txt`.


## Compilar, flashear y monitorear

1) Cargar entorno de ESP-IDF (Linux):

```bash
source $IDF_PATH/export.sh
```

2) (Opcional) Seleccionar target si fuera necesario:

```bash
idf.py set-target esp32
```

3) Compilar:

```bash
idf.py build
```

4) Flashear y abrir monitor serie (ajusta el puerto):

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Sugerencia: Para salir del monitor de ESP-IDF usa `Ctrl-]`.


## Uso desde la consola UART

La tarea `motor_console_task` abre UART0 a 115200 baudios. Puedes interactuar por el mismo puerto de programación/monitor.

Comandos disponibles:

- `T<grados>`
	- Fija el target de ángulo en grados. Ejemplos: `T30`, `T-45.5`

- `PID <P> <I> <D>`
	- Ajusta el PID de velocidad (en rad/s). Ejemplo: `PID 0.5 0.1 0.0`
	- Resetea el integral al aplicar nuevos parámetros para evitar windup.

- `MOD <tipo>` o `MODULATION <tipo>`
	- Cambia la modulación FOC. Opciones: `SinePWM`, `SpaceVectorPWM`, `Trapezoid_120`, `Trapezoid_150`
	- Atajos aceptados: `sine`, `svpwm`, `trap120`, `trap150`

- `S`
	- Alterna estado del motor (enable/disable).

- `SHOW` o `STATUS`
	- Muestra target, PID actual, tipo de modulación y estado del motor.

- `H` / `HELP`
	- Muestra ayuda rápida.


## Parámetros por defecto (ajústalos a tu hardware)

En `motor_control_task`:

- Fuente y límites:
	- `driver.voltage_power_supply = 9.0`
	- `driver.voltage_limit = 5.0`
	- `motor.voltage_limit = 5.0`
	- `motor.voltage_sensor_align = 8.0` (calibración)
- Control:
	- `motor.controller = MotionControlType::angle`
	- `motor.torque_controller = TorqueControlType::voltage`
	- `motor.foc_modulation = FOCModulationType::SpaceVectorPWM`
	- `motor.velocity_limit = 360 deg/s` (en rad/s internamente)
- Filtros y PID (velocidad):
	- `motor.LPF_angle = 0.001f`
	- `motor.LPF_velocity = 0.05f`
	- `P = 0.5`, `I = 0.0`, `D = 0.0`

Recomendación: sube/baja `voltage_limit` con precaución y verifica temperatura de la etapa de potencia.


## Arquitectura rápida

- `motor_control_task`: configura motor/driver, calibra FOC y ejecuta el lazo principal (`loopFOC` + `move` a `g_target_angle_rad`).
- `motor_console_task`: interpreta comandos por UART y actualiza parámetros/objetivos en tiempo real.
- `motor_debug_task`: imprime telemetría (comentada por defecto). Actívala si la necesitas.


## Interfaz gráfica en Python (WIP)

Próximamente: una GUI para controlar posición, PID, modulación y visualizar telemetría.

- Ubicación prevista: `scripts/` (por ejemplo `scripts/gui_control.py`).
- Requisitos (instalar):

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r scripts/requirements.txt
```

- Objetivos de la GUI:
	- Slider/inputs para posición objetivo (grados).
	- Campos para PID (P/I/D) y selector de modulación.
	- Gráficas en tiempo real de ángulo/velocidad.
	- Comunicación serie (pyserial) con los mismos comandos que la consola.

Estado: pendiente de implementación. Este README se actualizará cuando esté disponible.


## Solución de problemas

- El motor vibra o no arranca tras la calibración:
	- Verifica pares de polos (`POLE_PAIRS`) y cableado de fases.
	- Revisa tensión de alimentación y límites de voltaje configurados.
	- Asegura masa común y señales I2C correctas (SDA/SCL, pull-ups si aplica).

- Lectura de ángulo extraña o ruidosa:
	- Comprueba distancia e imán del AS5600, ruido EMI y cables I2C.
	- Ajusta filtros `LPF_angle`/`LPF_velocity` con moderación.

- No responde a comandos:
	- Confirma el puerto serie y baudios (115200) en el monitor.
	- Revisa que la tarea de consola está creada (ver logs).


## Seguridad

Motores BLDC pueden moverse de forma inesperada. Sujeta firmemente el motor, usa protecciones, monitorea temperatura y evita operar sin supervisión.


## Estructura del proyecto (resumen)

- `main/`
	- `main.cpp`: lógica principal (FOC, consola, tareas FreeRTOS)
	- `idf_component.yml`: componentes administrados (esp-simplefoc, etc.)
- `scripts/`
	- Utilidades Python (control, plotting, GUI WIP)
- `build/`
	- Artefactos generados por CMake/IDF (no editar)


## Próximos pasos sugeridos

- Completar la GUI en Python en `scripts/` y documentarla aquí.
- Exponer parámetros adicionales (p. ej., límites de voltaje/velocidad) como comandos.
- Añadir una opción para activar `motor_debug_task` desde consola.
- Documentar la placa/target exacto de ESP32 usada (si difiere de `esp32`).


---

Si tienes dudas o quieres extender el proyecto (corriente/torque closed-loop, perfiles de movimiento, etc.), puedes abrir issues o PRs. ¡Buen control FOC!

