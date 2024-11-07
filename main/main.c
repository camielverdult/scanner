#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "RSSI_SENSOR"
#define RSSI_THRESHOLD -70  // RSSI threshold to trigger light
#define LED_GPIO GPIO_NUM_2 // Define your LED GPIO here

// Callback for BLE scan results
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            // Check result for the following events:
            // 0: Inquiry result for a peer device.
            // 1: Inquiry complete.
            // 2: Discovery result for a peer device.
            // 3: Discovery result for BLE GATT based service on a peer device.
            // 4: Discovery complete.
            // 5: Discovery complete.
            if (scan_result->scan_rst.search_evt <= ESP_GAP_SEARCH_DI_DISC_CMPL_EVT) {
                int rssi = scan_result->scan_rst.rssi;
                ESP_LOGI(TAG, "Device found with RSSI: %d", rssi);
                
                // Trigger LED based on RSSI threshold
                if (rssi > RSSI_THRESHOLD) {
                    // gpio_set_level(LED_GPIO, 1);  // Turn on LED if within range
                    ESP_LOGI(TAG, "Device in range");
                } else {
                    // gpio_set_level(LED_GPIO, 0);  // Turn off LED if out of range
                    ESP_LOGI(TAG, "Device out of range");
                }
            }
            break;
        }
        default:
            break;
    }
}

void app_main(void) {
#if CONFIG_IDF_TARGET_ESP32C3
	// if GPIO8 is low at power on, it will never connect with the same 
	// debug log of this topic (USB-JTAG embedded port).
	// if GPIO8 is high it works.
	// set pin D8 state to high during boot 
	//zero-initialize the config structure.
    gpio_config_t io_conf = {};

	//disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set
	io_conf.pin_bit_mask = (1ULL << GPIO_NUM_8);
	//disable pull-down mode
    io_conf.pull_down_en = 0;
	//enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
#endif // CONFIG_IDF_TARGET_ESP32C3

    // Initialize non-volatile storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize the ESP32 BLE Controller
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Initialize Bluedroid stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register the GAP callback function
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    // Set BLE scan parameters
    esp_ble_scan_params_t ble_scan_params = {
        .scan_type              = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval          = 1600, // ms
        .scan_window            = 30    // ms
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
}
