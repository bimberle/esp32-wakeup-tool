#include "usb.h"
#include "tof_sensor.h"
#include "main.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "main";

void app_main(void) {
	esp_log_level_set("*", ESP_LOG_INFO);

	ESP_LOGI(TAG, "=== ESP32-S3 Motion Wake Tool ===");
	ESP_LOGI(TAG, "VL53L0X ToF Sensor -> USB HID Wake");

	usb_init();
	tof_sensor_init();  // ToF + LEDs

	ESP_LOGI(TAG, "System bereit. Warte auf Bewegung...");

	vTaskSuspend(NULL);
}
