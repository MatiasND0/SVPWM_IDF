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


// Funcion como control de volumen 
// Se lee el encoder y se ajusta el volumen en consecuencia
// se dispone un limite de +- 90 grados desde la posicion central (0 grados)
// superados este valor el volumen se mantiene en el maximo o minimo
void encoder_with_limit()
{
    // Leer el valor del encoder
    float encoder_value = sensor.getAngle();

    // Convertir a grados
    encoder_value = (encoder_value * 180.0f / 3.14159265359f);

    if (encoder_value > 360.0f) {
        encoder_value = 0.0f;
    } else if (encoder_value < -360.0f) {
        encoder_value = 0.0f;
    }

    // Aplicar límite de +- 90 grados
    if (encoder_value > 90.0f) {
        encoder_value = 90.0f;
    } else if (encoder_value < -90.0f) {
        encoder_value = -90.0f;
    }
    //Mapea -90 a 90 grados a volumen 0 a 100
    float volume = (encoder_value + 90.0f) * (100.0f / 180.0f);
    ESP_LOGI(TAG, "Valor del encoder con límite aplicado: %.2f grados", encoder_value);

    // Ajustar la posicion del motor fuera del rango
    if (encoder_value == 90.0f) {
        motor.move(90.0f * 3.14159265359f / 180.0f);
    } else if (encoder_value == -90.0f) {
        motor.move(-90.0f * 3.14159265359f / 180.0f);
    }
    

    // Si el motor esta en el rango apago el motor para ahorrar energia
    if (motor_running && (fabsf(encoder_value) < 90.0f)) {
        motor.disable();
        motor_running = false;
        ESP_LOGI(TAG, "Motor detenido para ahorrar energía (dentro del rango).");
    }
    // Reactivar el motor si el encoder se sale del rango
    else if (!motor_running && (fabsf(encoder_value) >= 90.0f)) {
        motor.enable();
        motor_running = true;
        ESP_LOGI(TAG, "Motor reactivado (fuera del rango).");
    }

}

void motor_control_task(void *pvParameters)
{
    // Configuraciones del motor
    driver.voltage_power_supply = 9.0; // Voltaje del bus (ej. 6V, 12V, 24V)
    driver.voltage_limit = 9.0;        // Límite de voltaje que el driver puede aplicar
    motor.voltage_limit = 9.0;         // Límite de voltaje del control del motor

    motor.voltage_sensor_align = 9.0; // 8 Volts para la calibración

    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;     // controlador de torque por voltaje
    motor.foc_modulation = FOCModulationType::SinePWM; // usar SVPWM con 6PWM
    motor.velocity_limit = 180.0f * 3.14159265359f / 180.0f;  // 720 deg/s en rad/s

    // Limitar el torque máximo (voltaje) aplicado al motor
    

    // Inicialización y calibración FOC
    ESP_LOGI(TAG, "Iniciando calibración FOC (el motor debe moverse)...");
    motor.init();
    motor.initFOC();
    ESP_LOGI(TAG, "Calibración FOC terminada.");

    // Bucle principal de control del motor
    while (1)
    {

        // 1) Ejecuta los algoritmos FOC
        motor.loopFOC();
        // // 2) Mueve hacia el objetivo de posición establecido por consola
        // motor.move(g_target_angle_rad);

        // // Si el motor esta proximo al target, apagarlo para ahorrar energia
        // if (motor_running && (fabsf(motor.shaft_angle - g_target_angle_rad) < (15.0f * 3.14159265359f / 180.0f))) {
        //     motor.disable();
        //     motor_running = false;
        //     ESP_LOGI(TAG, "Motor detenido para ahorrar energía (cercano al target).");
        // }
        // // Reactivar el motor si el target se aleja suficientemente
        // else if (!motor_running && (fabsf(motor.shaft_angle - g_target_angle_rad) >= (15.0f * 3.14159265359f / 180.0f))) {
        //     motor.enable();
        //     motor_running = true;
        //     ESP_LOGI(TAG, "Motor reactivado (target alejado).");
        // }

        encoder_with_limit();

        vTaskDelay(pdMS_TO_TICKS(1));
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

    ESP_LOGI(TAG, "Consola lista. Envíe comandos como T30 (grados).");

    char lineBuf[128];
    size_t lineLen = 0;

    auto process_line = [&](const char *line) {
        // Saltar espacios iniciales
        while (*line && isspace((unsigned char)*line)) line++;
        if (*line == '\0') return; // línea vacía

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
            ESP_LOGI(TAG, "Comandos:\n  T<grados>  -> fija el target de ángulo (ej: T-45.5)\n  H/HELP     -> muestra esta ayuda");
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