#include "main.h"

#include <tinyusb.h>
#include <class/hid/hid_device.h>
#include <esp_log.h>
#include <event_groups.h>

static const char *TAG = "usb";

#define USB_EVENT_KEYPRESS	(1 << 0)
static EventGroupHandle_t usb_event_group = NULL;

// Track USB state for debugging
static volatile bool usb_suspended = false;
static volatile bool remote_wakeup_enabled = false;

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

const uint8_t hid_report_descriptor[] = {
	TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD))
};

const char *hid_string_descriptor[5] = {
	// Array of pointer to string descriptors
	(char[]) {
 0x09, 0x04
},  // 0: is supported language is English (0x0409)
"ESP",						// 1: Manufacturer
"Wakeup Keyboard Device",	// 2: Product
"123456",					// 3: Serials, should use chip ID
"Example HID interface",	// 4: HID
};

static const uint8_t hid_configuration_descriptor[] = {
	// Configuration number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

	// Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
	TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
	// We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
	return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
	(void)instance;
	(void)report_id;
	(void)report_type;
	(void)buffer;
	(void)reqlen;

	return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
}

// ============ USB Suspend/Resume Callbacks ============
// Called when USB host suspends the device (e.g. Windows goes to sleep)
void tud_suspend_cb(bool remote_wakeup_en) {
	usb_suspended = true;
	remote_wakeup_enabled = remote_wakeup_en;
	ESP_LOGI(TAG, "USB SUSPENDED! Remote wakeup %s", 
		remote_wakeup_en ? "ENABLED ✓" : "DISABLED ✗");
}

// Called when USB host resumes the device
void tud_resume_cb(void) {
	usb_suspended = false;
	ESP_LOGI(TAG, "USB RESUMED");
}

// Called when device is mounted (configured) by host
void tud_mount_cb(void) {
	ESP_LOGI(TAG, "USB MOUNTED - Device configured by host");
}

// Called when device is unmounted
void tud_umount_cb(void) {
	ESP_LOGI(TAG, "USB UNMOUNTED");
}

void usb_request_keypress_send(bool from_isr) {
	if (from_isr)
		xEventGroupSetBitsFromISR(usb_event_group, USB_EVENT_KEYPRESS, NULL);
	else
		xEventGroupSetBits(usb_event_group, USB_EVENT_KEYPRESS);
}

static void usb_task(void *pvParameters) {
	ESP_LOGI(TAG, "usb task started");

	while (1) {
		EventBits_t bits = xEventGroupWaitBits(usb_event_group, USB_EVENT_KEYPRESS, pdTRUE, pdFALSE, portMAX_DELAY);
		if (bits & USB_EVENT_KEYPRESS) {
			xEventGroupClearBits(usb_event_group, USB_EVENT_KEYPRESS);

			// Debug: Show current USB state
			bool mounted = tud_mounted();
			bool suspended = tud_suspended();
			ESP_LOGI(TAG, "Wake request! mounted=%d, suspended=%d, remote_wakeup_en=%d", 
				mounted, suspended, remote_wakeup_enabled);

			if (suspended && remote_wakeup_enabled) {
				// Device is suspended and remote wakeup is allowed - send wake signal
				ESP_LOGI(TAG, "🔔 Sending USB REMOTE WAKEUP signal!");
				bool success = tud_remote_wakeup();
				if (success) {
					ESP_LOGI(TAG, "✓ Remote wakeup signal sent successfully");
				} else {
					ESP_LOGW(TAG, "✗ Remote wakeup failed - host may not support it");
				}
			} else if (suspended && !remote_wakeup_enabled) {
				ESP_LOGW(TAG, "⚠ Device suspended but remote wakeup NOT enabled by host!");
				ESP_LOGW(TAG, "   Check Windows Device Manager: Keyboard -> Power Management");
				ESP_LOGW(TAG, "   Enable: 'Allow this device to wake the computer'");
			} else if (mounted) {
				// Not suspended, send a keypress instead
				ESP_LOGI(TAG, "Not suspended, sending keypress");
				uint8_t keycode[6] = { HID_KEY_SPACE };
				tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
				vTaskDelay(pdMS_TO_TICKS(50));
				tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
			} else {
				ESP_LOGW(TAG, "USB not ready - not mounted and not suspended");
			}
		}
	}
}

void usb_init(void) {
	ESP_LOGI(TAG, "usb init");
	usb_event_group = xEventGroupCreate();
	const tinyusb_config_t tusb_cfg = {
		.device_descriptor = NULL,
		.string_descriptor = hid_string_descriptor,
		.string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
		.external_phy = false,
		.configuration_descriptor = hid_configuration_descriptor,
	};

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
	if (xTaskCreate(usb_task, "usb_task", 4096, NULL, 5, NULL) != pdPASS)
		ESP_LOGE(TAG, "error creating usb task");
	ESP_LOGI(TAG, "usb init done");
}
