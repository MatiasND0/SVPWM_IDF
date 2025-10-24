#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_simplefoc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <cctype>
#include <cstdio>
#include <strings.h>

// Helper: readable name for modulation type
static const char* foc_modulation_name(FOCModulationType m) {
    switch (m) {
        case FOCModulationType::SinePWM: return "SinePWM";
        case FOCModulationType::SpaceVectorPWM: return "SpaceVectorPWM";
        case FOCModulationType::Trapezoid_120: return "Trapezoid_120";
        case FOCModulationType::Trapezoid_150: return "Trapezoid_150";
        default: return "Unknown";
    }
}

// --- 1. Definiciones del Hardware ---
#define PIN_UH 25
#define PIN_UL 26
#define PIN_VH 27
#define PIN_VL 14
#define PIN_WH 32
#define PIN_WL 33
#define POLE_PAIRS 7

// Pines del Sensor
#define PIN_SDA 21
#define PIN_SCL 22

static const char *TAG = "MOTOR_FOC_TORQUE"; // NUEVO: Tag actualizado

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
AS5600 sensor = AS5600(I2C_NUM_0, (gpio_num_t)PIN_SCL, (gpio_num_t)PIN_SDA);

// Target de ángulo global (radianes), actualizado desde consola
static volatile float g_target_angle_rad = 0.0f;

// Estado del motor
bool motor_running = true;

void motor_control_task(void *pvParameters)
{
    // Configuraciones del motor
    driver.voltage_power_supply = 9.0; // Voltaje del bus (ej. 6V, 12V, 24V)
    driver.voltage_limit = 5.0;        // Límite de voltaje que el driver puede aplicar
    motor.voltage_limit = 5.0;         // Límite de voltaje del control del motor

    motor.voltage_sensor_align = 8.0; // 8 Volts para la calibración

    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;     // controlador de torque por voltaje
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // usar SVPWM con 6PWM
    motor.velocity_limit = 360.0f * 3.14159265359f / 180.0f;  // 360 deg/s en rad/s

    motor.LPF_angle = 0.001f;    // Filtro paso bajo para el ángulo
    motor.LPF_velocity = 0.05f;  // Filtro paso bajo para la velocidad

    motor.PID_velocity.P = 0.5f; // Controlador PID de velocidad
    motor.PID_velocity.I = 0.0f; // Controlador PID de velocidad
    motor.PID_velocity.D = 0.0f; // Controlador PID de velocidad

    // Inicialización y calibración FOC
    ESP_LOGI(TAG, "Iniciando calibración FOC (el motor debe moverse)...");
    motor.init();
    motor.initFOC();
    ESP_LOGI(TAG, "Calibración FOC terminada.");
    // Bucle principal de control del motor
    while (1)
    {
        motor.loopFOC();
        motor.move(g_target_angle_rad); // Usa el target global actualizado desde consola
    }
}

void motor_debug_task(void *pvParameters)
{
    while (1)
    {
        // Imprime datos de depuración cada 500ms
        ESP_LOGI(TAG, "Angle: %.2f rad, Velocity: %.2f rad/s, Target: %.2f rad",
                motor.shaft_angle,
                motor.shaft_velocity,
                g_target_angle_rad);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void motor_console_task(void *pvParameters)
{
    // Configurar UART0 para lectura de comandos (115200-8-N-1)
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // No cambiamos pines: usamos los del console por defecto
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Buffer de RX de 1024 bytes, sin TX buffer
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "Consola lista. Comandos: T<deg>, PID <P> <I> <D>, MOD <type>, SHOW, H.");

    char lineBuf[128];
    size_t lineLen = 0;

    auto process_line = [&](const char *line) {
        // Saltar espacios iniciales
        while (*line && isspace((unsigned char)*line)) line++;
        if (*line == '\0') return; // línea vacía

        // PID configuración: PID <P> <I> <D>
        if (strncasecmp(line, "PID", 3) == 0) {
            line += 3;
            while (*line && isspace((unsigned char)*line)) line++;
            float p = 0.f, i = 0.f, d = 0.f;
            int n = sscanf(line, "%f %f %f", &p, &i, &d);
            if (n == 3) {
                motor.PID_velocity.P = p;
                motor.PID_velocity.I = i;
                motor.PID_velocity.D = d;
                // Reset integral to avoid windup across parameter changes
                motor.PID_velocity.reset();
                ESP_LOGI(TAG, "PID velocity actualizado: P=%.4f I=%.4f D=%.4f", p, i, d);
            } else {
                ESP_LOGW(TAG, "Uso PID inválido. Formato: PID <P> <I> <D> (ej: PID 0.5 0.1 0.0)");
            }
            return;
        }

        // Modulación: MOD <SinePWM|SpaceVectorPWM|Trapezoid_120|Trapezoid_150>
        if (strncasecmp(line, "MODULATION", 10) == 0 || strncasecmp(line, "MOD", 3) == 0) {
            // avanzar hasta después del comando
            if (strncasecmp(line, "MODULATION", 10) == 0) line += 10; else line += 3;
            while (*line && isspace((unsigned char)*line)) line++;
            char typ[32] = {0};
            // capturar la primera palabra (tipo)
            int len = 0;
            while (*line && !isspace((unsigned char)*line) && len < (int)sizeof(typ)-1) {
                typ[len++] = *line++;
            }
            typ[len] = '\0';
            if (len == 0) {
                ESP_LOGW(TAG, "Uso MOD inválido. Formato: MOD <SinePWM|SpaceVectorPWM|Trapezoid_120|Trapezoid_150>");
                return;
            }

            FOCModulationType new_mod = motor.foc_modulation; // default to current
            if (strcasecmp(typ, "SinePWM") == 0 || strcasecmp(typ, "sine") == 0) {
                new_mod = FOCModulationType::SinePWM;
            } else if (strcasecmp(typ, "SpaceVectorPWM") == 0 || strcasecmp(typ, "svpwm") == 0) {
                new_mod = FOCModulationType::SpaceVectorPWM;
            } else if (strcasecmp(typ, "Trapezoid_120") == 0 || strcasecmp(typ, "trap120") == 0) {
                new_mod = FOCModulationType::Trapezoid_120;
            } else if (strcasecmp(typ, "Trapezoid_150") == 0 || strcasecmp(typ, "trap150") == 0) {
                new_mod = FOCModulationType::Trapezoid_150;
            } else {
                ESP_LOGW(TAG, "Tipo de modulación desconocido: %s", typ);
                ESP_LOGW(TAG, "Opciones: SinePWM, SpaceVectorPWM, Trapezoid_120, Trapezoid_150");
                return;
            }

            motor.foc_modulation = new_mod;
            ESP_LOGI(TAG, "Modulación FOC: %s", foc_modulation_name(new_mod));
            return;
        }

        // Mostrar configuración actual
        if (strncasecmp(line, "SHOW", 4) == 0 || strncasecmp(line, "STATUS", 6) == 0) {
            ESP_LOGI(TAG, "CONFIG -> Target: %.3f rad | PIDv: P=%.4f I=%.4f D=%.4f | Mod=%s | Motor=%s",
                    g_target_angle_rad,
                    motor.PID_velocity.P, motor.PID_velocity.I, motor.PID_velocity.D,
                    foc_modulation_name(motor.foc_modulation),
                    motor_running ? "ENABLED" : "DISABLED");
            return;
        }

        if (*line == 'T' || *line == 't') {
            line++;
            // Permitir espacios entre T y número
            while (*line && isspace((unsigned char)*line)) line++;
            // Parsear número en grados
            float deg = 0.0f;
            if (sscanf(line, "%f", &deg) == 1) {
                const float rad = deg * 3.14159265359f / 180.0f;
                g_target_angle_rad = rad; // escribir variable global
                ESP_LOGI(TAG, "Nuevo target: %.2f deg (%.3f rad)", deg, rad);
            } else {
                ESP_LOGW(TAG, "Comando T inválido. Uso: T<grados>, ej: T30");
            }
            return;
        }

        // Estado del motor
        if (*line == 'S' || *line == 's') {
            motor_running = !motor_running;
            if (motor_running) {
                motor.enable();
            } else {
                motor.disable();
            }
            ESP_LOGI(TAG, "Estado del motor: %s", motor_running ? "En marcha" : "Detenido");
            return;
        }

        // Ayuda básica
        if (*line == 'H' || *line == 'h' || strncasecmp(line, "help", 4) == 0) {
            ESP_LOGI(TAG,
                "Comandos:\n"
                "  T<grados>            -> Fija el target de ángulo (ej: T-45.5)\n"
                "  PID <P> <I> <D>      -> Ajusta el PID de velocidad (ej: PID 0.5 0.1 0.0)\n"
                "  MOD <tipo>           -> Modulación FOC: SinePWM | SpaceVectorPWM | Trapezoid_120 | Trapezoid_150\n"
                "  S                    -> Alterna estado del motor (enable/disable)\n"
                "  SHOW                 -> Muestra la configuración actual\n"
                "  H/HELP               -> Muestra esta ayuda");
            return;
        }

        ESP_LOGW(TAG, "Comando no reconocido. Use 'H' para ayuda.");
    };

    while (1) {
        uint8_t rx[64];
        int rxlen = uart_read_bytes(uart_num, rx, sizeof(rx), pdMS_TO_TICKS(50));
        if (rxlen > 0) {
            for (int i = 0; i < rxlen; ++i) {
                char c = (char)rx[i];
                if (c == '\r' || c == '\n') {
                    if (lineLen > 0) {
                        lineBuf[lineLen] = '\0';
                        process_line(lineBuf);
                        lineLen = 0;
                    }
                } else {
                    if (lineLen + 1 < sizeof(lineBuf)) {
                        lineBuf[lineLen++] = c;
                    } else {
                        // línea muy larga: procesar lo que haya y reiniciar
                        lineBuf[lineLen] = '\0';
                        process_line(lineBuf);
                        lineLen = 0;
                    }
                }
            }
        }
        // Pequeña espera para ceder CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando control FOC de Posicion...");

    // Inicializa el sensor
    sensor.init();
    ESP_LOGI(TAG, "Sensor AS5600 inicializado.");

    // Inicializa el driver
    driver.init();
    ESP_LOGI(TAG, "Driver (MCPWM) inicializado.");

    // Vinculación del motor, driver y sensor
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);

    // Crea la tarea de control del motor
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tarea FOC creada.");
    // Crea la tarea de debug del motor
    // xTaskCreate(motor_debug_task, "motor_debug_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tarea de depuración creada.");
    // Crea la tarea de comando por consola (opcional)
    xTaskCreate(motor_console_task, "motor_console_task", 8192, NULL, 5, NULL);

    
}