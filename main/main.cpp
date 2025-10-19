#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_simplefoc.h"
#include "esp_log.h"
#include "esp_timer.h"

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

// --- 2. Objetos Globales ---
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
AS5600 sensor = AS5600(I2C_NUM_0, (gpio_num_t)PIN_SCL, (gpio_num_t)PIN_SDA);

// --- 3. Tarea de Control del Motor ---
void motor_control_task(void *pvParameters)
{

    // Ajusta estos valores a tu fuente real
    driver.voltage_power_supply = 9.0; // Voltaje del bus (ej. 6V, 12V, 24V)
    driver.voltage_limit = 4.0;        // Límite de voltaje que el driver puede aplicar
    motor.voltage_limit = 4.0;         // Límite de voltaje del control del motor

    // NUEVO: Ajustar el voltaje de alineación
    // Si el motor no "patea" fuerte al inicio, sube este valor.
    motor.voltage_sensor_align = 8.0; // 8 Volts para la calibración

    // NUEVO: Establecer modo de control de torque
    // Control de posición con límite de velocidad
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;     // controlador de torque por voltaje
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // usar SVPWM con 6PWM
    motor.velocity_limit = 720.0f * 3.14159265359f / 180.0f;  // 720 deg/s en rad/s

    // NUEVO: ¡Calibración FOC!
    // El motor se moverá aquí.
    ESP_LOGI(TAG, "Iniciando calibración FOC (el motor debe moverse)...");
    motor.init();
    motor.initFOC();
    ESP_LOGI(TAG, "Calibración FOC terminada.");

    // NUEVO: Voltaje objetivo (torque)
    // Objetivo de posición que avanza a 30°/s
    float target_angle = sensor.getAngle();              // arranca desde el ángulo actual para evitar saltos
    int count = 0;
    while (1)
    {
        // --- Bucle FOC Rápido ---

        // 1) Ejecuta los algoritmos FOC
        motor.loopFOC();
        count++;
        // 3) Mueve hacia el objetivo de posición
        if (count < 5000) { // Avanza 1 radian cada segundo
            motor.move(3.14159265359f);
        }
        else {
            motor.move(0);
            if (count >= 10000) {
                count = 0; // Reinicia el contador cada 10 segundos
            }
        }         

        // NUEVO: Delay de 1ms para un bucle FOC de 1kHz
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- 4. Función Principal (app_main) ---
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando control FOC de Torque...");

    // Inicializa el sensor
    sensor.init();
    ESP_LOGI(TAG, "I2C y Sensor AS5600 inicializados.");

    // Comprobación rápida del sensor: leer unos ángulos
    for (int i = 0; i < 5; i++)
    {
        float ang = sensor.getMechanicalAngle();
        ESP_LOGI(TAG, "AS5600 angle rad=%.4f", ang);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Inicializa el driver
    driver.init();
    ESP_LOGI(TAG, "Driver (MCPWM) inicializado.");

    // --- Vinculación completa ---
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor); // Cierra el lazo con el sensor

    xTaskCreate(
        motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tarea FOC creada. El motor debería avanzar 10°/s.");
}