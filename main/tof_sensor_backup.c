/**
 * VL53L0X ToF Sensor für Motion Detection
 * Triggeriert USB Wakeup bei Bewegung
 */

#include "tof_sensor.h"
#include "usb.h"
#include "main.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "tof_sensor";

#define VL53L0X_I2C_ADDR            0x29
#define VL53L0X_REG_IDENTIFICATION  0xC0
#define VL53L0X_REG_SYSRANGE_START  0x00
#define VL53L0X_REG_RESULT_RANGE    0x14

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_TIMEOUT_MS              1000

static uint16_t last_distance = 0;
static TickType_t last_motion_time = 0;
static bool in_cooldown = false;

// LED Hilfsfunktionen (nur GPIO2)
static void led_init_simple(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_ON_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_ON_PIN, 0);
}

static void led_blink_motion(void) {
    // Kurzes Blinken bei Bewegung (3x schnell)
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_ON_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(LED_ON_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// I2C initialisieren
static esp_err_t i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOF_I2C_SDA,
        .scl_io_num = TOF_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Register lesen
static esp_err_t vl53l0x_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, VL53L0X_I2C_ADDR, &reg, 1, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Register schreiben
static esp_err_t vl53l0x_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, VL53L0X_I2C_ADDR, buf, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Einzelmessung starten und Distanz lesen (vereinfacht)
static uint16_t vl53l0x_read_distance(void) {
    uint8_t data[2];
    
    // Messung starten
    vl53l0x_write_reg(VL53L0X_REG_SYSRANGE_START, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Ergebnis lesen (Register 0x14 + 10/11 = Range in mm)
    if (vl53l0x_read_reg(0x14 + 10, data, 2) != ESP_OK) {
        return 0xFFFF;  // Fehler
    }
    
    return (data[0] << 8) | data[1];
}

// Sensor-Task: Liest kontinuierlich und triggert Wake bei Bewegung
static void tof_sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "ToF Sensor Task gestartet");
    
    while (1) {
        uint16_t distance = vl53l0x_read_distance();
        
        if (distance < 2000 && distance != 0xFFFF) {
            ESP_LOGI(TAG, "Distanz: %u mm", distance);
            
            // Bewegungserkennung
            if (last_distance > 0) {
                int delta = abs((int)distance - (int)last_distance);
                
                if (delta > MOTION_THRESHOLD_MM) {
                    TickType_t now = xTaskGetTickCount();
                    TickType_t cooldown_ticks = pdMS_TO_TICKS(MOTION_COOLDOWN_MS);
                    
                    if (!in_cooldown || (now - last_motion_time > cooldown_ticks)) {
                        ESP_LOGI(TAG, "🚶 Bewegung erkannt! Δ=%d mm -> USB Wake!", delta);
                        led_blink_motion();  // LED blinken!
                        usb_request_keypress_send(false);
                        last_motion_time = now;
                        in_cooldown = true;
                    } else {
                        ESP_LOGD(TAG, "Bewegung ignoriert (Cooldown)");
                    }
                }
            }
            
            // Cooldown beenden
            if (in_cooldown && (xTaskGetTickCount() - last_motion_time > pdMS_TO_TICKS(MOTION_COOLDOWN_MS))) {
                in_cooldown = false;
                ESP_LOGI(TAG, "Cooldown beendet");
            }
            
            last_distance = distance;
        } else {
            ESP_LOGW(TAG, "Sensor-Fehler oder außer Reichweite");
        }
        
        vTaskDelay(pdMS_TO_TICKS(TOF_SAMPLE_INTERVAL_MS));
    }
}

void tof_sensor_init(void) {
    ESP_LOGI(TAG, "Initialisiere VL53L0X ToF Sensor...");
    
    // LEDs initialisieren
    led_init_simple();
    ESP_LOGI(TAG, "LED initialisiert (GPIO2)");
    
    // Kurzes LED-Blinken beim Start
    led_blink_motion();
    
    // XSHUT Pin für Sensor aktivieren
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOF_XSHUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(TOF_XSHUT_PIN, 1);  // Sensor aktivieren
    vTaskDelay(pdMS_TO_TICKS(50));  // Mehr Zeit zum Aufwachen
    
    // I2C initialisieren
    esp_err_t err = i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C Init fehlgeschlagen: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Sensor nicht verfügbar - LED blinkt alle 2s als Test");
        // Test-Modus: Blinke alle 2 Sekunden
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            led_blink_motion();
            ESP_LOGI(TAG, "Test-Blink (kein Sensor)");
        }
    }
    
    // Sensor prüfen
    uint8_t id;
    err = vl53l0x_read_reg(VL53L0X_REG_IDENTIFICATION, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X nicht gefunden! I2C Fehler: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Sensor nicht verfügbar - LED blinkt alle 2s als Test");
        // Test-Modus: Blinke alle 2 Sekunden
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            led_blink_motion();
            ESP_LOGI(TAG, "Test-Blink (kein Sensor)");
        }
    }
    ESP_LOGI(TAG, "VL53L0X gefunden, ID: 0x%02X", id);
    
    // Sensor-Task starten
    xTaskCreate(tof_sensor_task, "tof_sensor", 4096, NULL, 5, NULL);
}
