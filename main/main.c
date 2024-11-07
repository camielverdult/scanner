#include <stdio.h>
#include <string.h>
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

// iPhone BT MAC is 30:D5:3E:06:4D:7D
static esp_bd_addr_t target_device_addr = {0x30, 0xD5, 0x3E, 0x06, 0x4D, 0x7D};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    .scan_interval = 0x10, // 10 ms
    .scan_window = 0x10, // 10 ms
};

void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void init_ble(void) {
    esp_err_t ret;

    // Initialize the Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return;
    }

    // Enable the Bluetooth controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    // Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return;
    }
}

static void get_device_name(esp_ble_gap_cb_param_t *param, char *name, int name_len)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    // Extract the device name from the advertisement data
    adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

    if (adv_name != NULL && adv_name_len > 0) {
        // Copy the device name to the output buffer
        snprintf(name, name_len, "%.*s", adv_name_len, adv_name);
    } else {
        snprintf(name, name_len, "No name");
    }
}

static void get_device_mac(esp_ble_gap_cb_param_t *param, char *mac, int mac_len)
{
    assert(mac_len >= 18);
    snprintf(mac, mac_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
             param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);
}

static void get_device_type(esp_ble_gap_cb_param_t *param, char *type, int type_len)
{
    assert(type_len >= sizeof("RPA_RANDOM"));
    switch (param->scan_rst.ble_addr_type) {
        case BLE_ADDR_TYPE_PUBLIC:
            snprintf(type, type_len, "PUBLIC");
            break;
        case BLE_ADDR_TYPE_RANDOM:
            snprintf(type, type_len, "RANDOM");
            break;
        case BLE_ADDR_TYPE_RPA_PUBLIC:
            snprintf(type, type_len, "RPA_PUBLIC");
            break;
        case BLE_ADDR_TYPE_RPA_RANDOM:
            snprintf(type, type_len, "RPA_RANDOM");
            break;
        default:
            snprintf(type, type_len, "UNKNOWN");
            break;
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Scan parameters set, starting scan...");
                esp_ble_gap_start_scanning(10);  // Scan for 10 seconds
            } else {
                ESP_LOGE(TAG, "Failed to set scan parameters");
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Get the device name
                char device_name[64];
                get_device_name(param, device_name, sizeof(device_name));

                // Get the MAC address
                char mac_address[18];
                get_device_mac(param, mac_address, sizeof(mac_address));
                // Find address type of the device
                char addr_type[12];
                get_device_type(param, addr_type, sizeof(addr_type));

                // Log the device name and RSSI
                ESP_LOGI(TAG, "%s device found: %s (%s), RSSI=%d", addr_type, device_name, mac_address, param->scan_rst.rssi);

                // Check if the device is the target device
                if (memcmp(param->scan_rst.bda, target_device_addr, ESP_BD_ADDR_LEN) == 0) {
                    ESP_LOGI(TAG, "Target device found");
                    break;
                }
            }
            break;

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan complete");
            break;

        default:
            break;
    }
}

void app_main(void) {
    // Initialize NVS
    init_nvs();

    // Initialize BLE
    init_ble();

    // Register GAP callback
    esp_ble_gap_register_callback(gap_event_handler);

    // Set scan parameters
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
    // esp_err_t err = esp_ble_gap_set_scan_params(&ble_scan_params);
    // if (ret == ESP_OK) {
    //     ESP_LOGI(TAG, "BLE scan parameters set successfully");
    // } else {
    //     ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
    // }
}
