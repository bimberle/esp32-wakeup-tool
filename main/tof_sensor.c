/**
 * VL53L0X ToF Sensor - Vollständiger Pololu-Port
 * 
 * Dieser Treiber ist eine direkte Portierung des Pololu VL53L0X Arduino Treibers
 * mit allen notwendigen Tuning-Settings und Kalibrierungen.
 */

#include "tof_sensor.h"
#include "usb.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "tof";

#define VL53L0X_ADDR    0x29
#define I2C_NUM         I2C_NUM_0
#define I2C_FREQ        100000
#define I2C_TIMEOUT     (100 / portTICK_PERIOD_MS)

// Register Adressen (aus Pololu VL53L0X.h)
#define SYSRANGE_START                              0x00
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define MSRC_CONFIG_CONTROL                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89
#define IDENTIFICATION_MODEL_ID                     0xC0

static uint16_t last_distance = MAX_DISTANCE_MM;  // Letzter gültiger Messwert
static TickType_t last_motion = 0;
static bool cooldown = false;
static uint8_t stop_variable = 0;

// I2C Funktionen
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_NUM, VL53L0X_ADDR, buf, 2, I2C_TIMEOUT);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t val = 0;
    i2c_master_write_read_device(I2C_NUM, VL53L0X_ADDR, &reg, 1, &val, 1, I2C_TIMEOUT);
    return val;
}

static uint16_t read_reg16(uint8_t reg) {
    uint8_t buf[2] = {0};
    i2c_master_write_read_device(I2C_NUM, VL53L0X_ADDR, &reg, 1, buf, 2, I2C_TIMEOUT);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void write_reg16(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    i2c_master_write_to_device(I2C_NUM, VL53L0X_ADDR, buf, 3, I2C_TIMEOUT);
}

static void write_multi(uint8_t reg, uint8_t *src, uint8_t count) {
    uint8_t buf[count + 1];
    buf[0] = reg;
    memcpy(&buf[1], src, count);
    i2c_master_write_to_device(I2C_NUM, VL53L0X_ADDR, buf, count + 1, I2C_TIMEOUT);
}

static void read_multi(uint8_t reg, uint8_t *dst, uint8_t count) {
    i2c_master_write_read_device(I2C_NUM, VL53L0X_ADDR, &reg, 1, dst, count, I2C_TIMEOUT);
}

// getSpadInfo - aus Pololu-Treiber
static bool get_spad_info(uint8_t *count, bool *type_is_aperture) {
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    
    write_reg(0xFF, 0x06);
    write_reg(0x83, read_reg(0x83) | 0x04);
    write_reg(0xFF, 0x07);
    write_reg(0x81, 0x01);
    
    write_reg(0x80, 0x01);
    
    write_reg(0x94, 0x6B);
    write_reg(0x83, 0x00);
    
    int timeout = 100;
    while (read_reg(0x83) == 0x00) {
        if (--timeout <= 0) {
            ESP_LOGE(TAG, "getSpadInfo timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    write_reg(0x83, 0x01);
    uint8_t tmp = read_reg(0x92);
    
    *count = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;
    
    write_reg(0x81, 0x00);
    write_reg(0xFF, 0x06);
    write_reg(0x83, read_reg(0x83) & ~0x04);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x01);
    
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);
    
    return true;
}

// performSingleRefCalibration - aus Pololu-Treiber
static bool perform_single_ref_calibration(uint8_t vhv_init_byte) {
    write_reg(SYSRANGE_START, 0x01 | vhv_init_byte);
    
    int timeout = 100;
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (--timeout <= 0) {
            ESP_LOGE(TAG, "RefCalibration timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_reg(SYSRANGE_START, 0x00);
    
    return true;
}

// Vollständige Init nach Pololu init()
static bool vl53l0x_init(void) {
    // Check ID
    uint8_t id = read_reg(IDENTIFICATION_MODEL_ID);
    ESP_LOGI(TAG, "Model ID: 0x%02X", id);
    if (id != 0xEE) {
        ESP_LOGE(TAG, "Wrong Model ID!");
        return false;
    }
    
    // VL53L0X_DataInit() begin
    
    // Sensor uses 1V8 mode for I/O by default; switch to 2V8 mode
    write_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 
              read_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    
    // "Set I2C standard mode"
    write_reg(0x88, 0x00);
    
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    stop_variable = read_reg(0x91);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);
    ESP_LOGI(TAG, "Stop variable: 0x%02X", stop_variable);
    
    // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    write_reg(MSRC_CONFIG_CONTROL, read_reg(MSRC_CONFIG_CONTROL) | 0x12);
    
    // Set final range signal rate limit to 0.25 MCPS (million counts per second)
    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    // 0.25 * 128 = 32 = 0x0020
    write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x0020);
    
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);
    
    // VL53L0X_DataInit() end
    
    // VL53L0X_StaticInit() begin
    
    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_spad_info(&spad_count, &spad_type_is_aperture)) {
        ESP_LOGE(TAG, "getSpadInfo failed");
        return false;
    }
    ESP_LOGI(TAG, "SPAD count: %d, aperture: %d", spad_count, spad_type_is_aperture);
    
    // Read SPAD map
    uint8_t ref_spad_map[6];
    read_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    
    // VL53L0X_set_reference_spads() begin
    write_reg(0xFF, 0x01);
    write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_reg(0xFF, 0x00);
    write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    
    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            spads_enabled++;
        }
    }
    
    write_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    // VL53L0X_set_reference_spads() end
    
    // VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h
    
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    
    write_reg(0xFF, 0x00);
    write_reg(0x09, 0x00);
    write_reg(0x10, 0x00);
    write_reg(0x11, 0x00);
    
    write_reg(0x24, 0x01);
    write_reg(0x25, 0xFF);
    write_reg(0x75, 0x00);
    
    write_reg(0xFF, 0x01);
    write_reg(0x4E, 0x2C);
    write_reg(0x48, 0x00);
    write_reg(0x30, 0x20);
    
    write_reg(0xFF, 0x00);
    write_reg(0x30, 0x09);
    write_reg(0x54, 0x00);
    write_reg(0x31, 0x04);
    write_reg(0x32, 0x03);
    write_reg(0x40, 0x83);
    write_reg(0x46, 0x25);
    write_reg(0x60, 0x00);
    write_reg(0x27, 0x00);
    write_reg(0x50, 0x06);
    write_reg(0x51, 0x00);
    write_reg(0x52, 0x96);
    write_reg(0x56, 0x08);
    write_reg(0x57, 0x30);
    write_reg(0x61, 0x00);
    write_reg(0x62, 0x00);
    write_reg(0x64, 0x00);
    write_reg(0x65, 0x00);
    write_reg(0x66, 0xA0);
    
    write_reg(0xFF, 0x01);
    write_reg(0x22, 0x32);
    write_reg(0x47, 0x14);
    write_reg(0x49, 0xFF);
    write_reg(0x4A, 0x00);
    
    write_reg(0xFF, 0x00);
    write_reg(0x7A, 0x0A);
    write_reg(0x7B, 0x00);
    write_reg(0x78, 0x21);
    
    write_reg(0xFF, 0x01);
    write_reg(0x23, 0x34);
    write_reg(0x42, 0x00);
    write_reg(0x44, 0xFF);
    write_reg(0x45, 0x26);
    write_reg(0x46, 0x05);
    write_reg(0x40, 0x40);
    write_reg(0x0E, 0x06);
    write_reg(0x20, 0x1A);
    write_reg(0x43, 0x40);
    
    write_reg(0xFF, 0x00);
    write_reg(0x34, 0x03);
    write_reg(0x35, 0x44);
    
    write_reg(0xFF, 0x01);
    write_reg(0x31, 0x04);
    write_reg(0x4B, 0x09);
    write_reg(0x4C, 0x05);
    write_reg(0x4D, 0x04);
    
    write_reg(0xFF, 0x00);
    write_reg(0x44, 0x00);
    write_reg(0x45, 0x20);
    write_reg(0x47, 0x08);
    write_reg(0x48, 0x28);
    write_reg(0x67, 0x00);
    write_reg(0x70, 0x04);
    write_reg(0x71, 0x01);
    write_reg(0x72, 0xFE);
    write_reg(0x76, 0x00);
    write_reg(0x77, 0x00);
    
    write_reg(0xFF, 0x01);
    write_reg(0x0D, 0x01);
    
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x01);
    write_reg(0x01, 0xF8);
    
    write_reg(0xFF, 0x01);
    write_reg(0x8E, 0x01);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);
    
    // VL53L0X_load_tuning_settings() end
    
    // "Set interrupt config to new sample ready"
    write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_reg(GPIO_HV_MUX_ACTIVE_HIGH, read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    // "Disable MSRC and TCC by default"
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    
    // VL53L0X_StaticInit() end
    
    // VL53L0X_PerformRefCalibration() begin
    
    // VL53L0X_perform_vhv_calibration() begin
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(0x40)) {
        ESP_LOGE(TAG, "VHV calibration failed");
        return false;
    }
    ESP_LOGI(TAG, "VHV calibration OK");
    
    // VL53L0X_perform_phase_calibration() begin
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(0x00)) {
        ESP_LOGE(TAG, "Phase calibration failed");
        return false;
    }
    ESP_LOGI(TAG, "Phase calibration OK");
    
    // "restore the previous Sequence Config"
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    
    // VL53L0X_PerformRefCalibration() end
    
    ESP_LOGI(TAG, "Init complete!");
    return true;
}

// Starte Continuous Mode
static void start_continuous(void) {
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, stop_variable);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);
    
    // Continuous back-to-back mode
    write_reg(SYSRANGE_START, 0x02);
    ESP_LOGI(TAG, "Continuous mode started");
}

// Lese Messung im Continuous Mode
static uint16_t read_continuous(void) {
    int timeout = 200;
    
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (--timeout <= 0) {
            return 0xFFFF;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Range: RESULT_RANGE_STATUS + 10 = 0x14 + 10 = 0x1E
    uint16_t range = read_reg16(RESULT_RANGE_STATUS + 10);
    
    // Clear interrupt
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    return range;
}

// Sensor Task - Einfache Delta-Erkennung
// >2m = nichts erkannt, bei Änderung >500mm = Motion
static void sensor_task(void *arg) {
    ESP_LOGI(TAG, "Task started - Delta detection (threshold: %d mm)", MOTION_THRESHOLD_MM);
    
    // Starte Continuous Mode
    start_continuous();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Main loop
    while (1) {
        uint16_t raw = read_continuous();
        
        // Aktueller Wert: >2m oder ungültig = MAX_DISTANCE ("nichts da")
        uint16_t current = (raw > 20 && raw < MAX_DISTANCE_MM) ? raw : MAX_DISTANCE_MM;
        
        int delta = abs((int)current - (int)last_distance);
        
        ESP_LOGI(TAG, "Dist: %u mm (raw: %u, last: %u, delta: %d)", 
                 current, raw, last_distance, delta);
        
        // Motion erkennen: große Änderung und nicht im Cooldown
        if (delta > MOTION_THRESHOLD_MM && !cooldown) {
            ESP_LOGI(TAG, ">>> MOTION! delta=%d mm <<<", delta);
            usb_request_keypress_send(false);
            last_motion = xTaskGetTickCount();
            cooldown = true;
        }
        
        // Cooldown Ende
        if (cooldown && (xTaskGetTickCount() - last_motion > pdMS_TO_TICKS(MOTION_COOLDOWN_MS))) {
            cooldown = false;
            ESP_LOGI(TAG, "Cooldown ended");
        }
        
        // Letzten Wert speichern
        last_distance = current;
        
        vTaskDelay(pdMS_TO_TICKS(TOF_SAMPLE_INTERVAL_MS));
    }
}

void tof_sensor_init(void) {
    ESP_LOGI(TAG, "Init (SDA=%d, SCL=%d, XSHUT=%d)", TOF_I2C_SDA, TOF_I2C_SCL, TOF_XSHUT_PIN);
    
    // XSHUT - Reset sensor
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TOF_XSHUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(TOF_XSHUT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOF_XSHUT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOF_I2C_SDA,
        .scl_io_num = TOF_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
    
    if (!vl53l0x_init()) {
        ESP_LOGE(TAG, "VL53L0X init FAILED!");
        return;
    }
    
    xTaskCreate(sensor_task, "tof", 4096, NULL, 5, NULL);
}
