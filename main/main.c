#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "RSSI_SENSOR"
#define DEVICE_NAME "ESP32C3 RSSI Sensor"
#define BLE_PASSKEY 123456  // Passkey for pairing
#define RSSI_THRESHOLD -70  // RSSI threshold to trigger light

#define MAX_BONDED_DEVICES 10

void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
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

static esp_bd_addr_t bonded_devices[MAX_BONDED_DEVICES];
static int num_bonded_devices = 0;

// Helper function to retrieve bonded devices from NVS
void get_bonded_devices(esp_bd_addr_t *bonded_devices, int *num_bonded) {
    int dev_num = esp_ble_get_bond_device_num();

    if (dev_num == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to get bonded device number");
        *num_bonded = 0;
        return;
    }

    if (dev_num == 0) {
        ESP_LOGI(TAG, "No bonded devices found");
        *num_bonded = 0;
        return;
    } else {
        ESP_LOGI(TAG, "Found %d bonded devices", dev_num);
    }

    // Allocate memory for the bonded device list
    esp_ble_bond_dev_t *bond_dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);

    // Get the bonded device list from NVS
    esp_ble_get_bond_device_list(&dev_num, bond_dev_list);

    *num_bonded = dev_num;
    for (int i = 0; i < dev_num; i++) {
        memcpy(bonded_devices[i], bond_dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
        // ESP_LOGI(TAG, "Bonded device %d: %02X:%02X:%02X:%02X:%02X:%02X", i,
        //          bonded_devices[i][0], bonded_devices[i][1], bonded_devices[i][2],
        //          bonded_devices[i][3], bonded_devices[i][4], bonded_devices[i][5]);
    }
    free(bond_dev_list);
}

// Check if the scanned device is a bonded device
bool is_bonded_device(esp_bd_addr_t bda) {
    for (int i = 0; i < num_bonded_devices; i++) {
        if (memcmp(bda, bonded_devices[i], ESP_BD_ADDR_LEN) == 0) {
            return true;
        }
    }
    return false;
}

// /* register three profiles, each profile corresponds to one connection,
//    which makes it easy to handle each connection event */
// #define PROFILE_NUM 3
// #define PROFILE_A_APP_ID 0
// #define PROFILE_B_APP_ID 1
// #define PROFILE_C_APP_ID 2

// /* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
// static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
//     [PROFILE_A_APP_ID] = {
//         .gattc_cb = gattc_profile_a_event_handler,
//         .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },
//     [PROFILE_B_APP_ID] = {
//         .gattc_cb = gattc_profile_b_event_handler,
//         .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },
//     [PROFILE_C_APP_ID] = {
//         .gattc_cb = gattc_profile_c_event_handler,
//         .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },

// };

// Set scan parameters
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    .scan_interval = 0x10, // 10 ms
    .scan_window = 0x10, // 10 ms
};

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        // Handle authentication events
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            ESP_LOGI(TAG, "Authentication completed with success: %d", param->ble_security.auth_cmpl.success);
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(TAG, "Device bonded successfully");
            }
            break;

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            ESP_LOGI(TAG, "Passkey requested");
            esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, BLE_PASSKEY);
            break;

        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                // Load bonded devices
                get_bonded_devices(bonded_devices, &num_bonded_devices);
                
                // Scan for devices
                ESP_LOGI(TAG, "Scan parameters set, starting scan...");
                esp_ble_gap_start_scanning(30);
            } else {
                ESP_LOGE(TAG, "Failed to set scan parameters");
            }
            break;

        // Handle scan events
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Check if the scanned device matches any bonded device address
                if (is_bonded_device(param->scan_rst.bda)) {
                    ESP_LOGI(TAG, "Bonded device found with RSSI=%d", param->scan_rst.rssi);
                    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());  // Stop scanning to initiate connection
                    ESP_ERROR_CHECK(esp_ble_gattc_open(NULL, param->scan_rst.bda, param->scan_rst.ble_addr_type, true));
                } else {
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
                }
            }
            break;

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan stopped/completed");
            break;

        case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
            if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                    ESP_LOGI(TAG, "RSSI of device %02X:%02X:%02X:%02X:%02X:%02X: %d dBm",
                            param->read_rssi_cmpl.remote_addr[0], param->read_rssi_cmpl.remote_addr[1],
                            param->read_rssi_cmpl.remote_addr[2], param->read_rssi_cmpl.remote_addr[3],
                            param->read_rssi_cmpl.remote_addr[4], param->read_rssi_cmpl.remote_addr[5],
                            param->read_rssi_cmpl.rssi);
                    
                    // Re-read RSSI periodically
                    esp_ble_gap_read_rssi(param->read_rssi_cmpl.remote_addr);
                } else {
                    ESP_LOGE(TAG, "Failed to read RSSI");
                }
            break;

        default:
            break;
    }
}

// Handle GATT client events for connection, disconnection, and RSSI tracking
void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    switch (event) {
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Connected to bonded device");
                esp_ble_gap_read_rssi(param->open.remote_bda);  // Read initial RSSI
            }
            break;

        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Disconnected from bonded device, attempting to reconnect...");
            // Attempt to reconnect to the bonded device
            esp_ble_gattc_open(gattc_if, param->disconnect.remote_bda, BLE_ADDR_TYPE_RANDOM, true);  // Reconnect
            break;

        default:
            break;
    }
}

void app_main(void) {
    // Initialize NVS
    init_nvs();

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

    // Set GAP parameters for pairing and bonding
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));

    // Enable address resolution
    ESP_ERROR_CHECK(esp_ble_gap_config_local_privacy(true));
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;  // Bonding with peer device
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req)));

    // Register GAP callback
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
                
    // Start scanning
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
}
