#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

#define TAG "BLE_SERVER"
#define DEVICE_NAME "ESP32_BLE_SERVER"

// HID service UUID (128-bit)
static uint8_t hid_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

// Advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,           // Include device name in main adv_data
    .include_txpower = false,       // Exclude TX power to save space
    .appearance = 0xC0,             // Generic HID device
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};// Some fields are not set to stay within the 31-byte limit.

/*
    Some advertising data (e.g., p_service_uuid) is moved from adv_data to 
    the separate scan response data structure below. 
    This allows scanning devices to still access the data 
    when they actively scan (request scan response).
*/
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = false,          // Name already in main adv_data
    .service_uuid_len = sizeof(hid_service_uuid),
    .p_service_uuid = hid_service_uuid,
};

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT service handle to add characteristics to
static uint16_t hid_service_handle = 0;

// Minimal HID Report Map for a Generic HID device
// static const uint8_t hid_report_map[] = {
//     0x05, 0x01,  // Usage Page (Generic Desktop)
//     0x09, 0x06,  // Usage (Keyboard)
//     0xA1, 0x01,  // Collection (Application)
//     0x05, 0x07,  // Usage Page (Key Codes)
//     0x19, 0xe0,  // Usage Minimum (224)
//     0x29, 0xe7,  // Usage Maximum (231)
//     0x15, 0x00,  // Logical Minimum (0)
//     0x25, 0x01,  // Logical Maximum (1)
//     0x75, 0x01,  // Report Size (1)
//     0x95, 0x08,  // Report Count (8)
//     0x81, 0x02,  // Input (Data, Variable, Absolute)
//     0xC0         // End Collection
// };

// HID Information characteristic (version 1.11, country code 0x00, flags 0x01)
static const uint8_t hid_information[] = { 0x11, 0x01, 0x00, 0x01 };  // Version 1.11, Country Code 0x00, Flags 0x01
static uint16_t hid_info_handle = 0;

// Store the address of the connected device
static esp_bd_addr_t connected_device_addr = {0};

// Timer to read RSSI periodically
static TimerHandle_t rssi_timer;

// RSSI read interval in milliseconds
#define READ_RSSI_TIMEOUT_MS 200

static float smoothed_rssi = 0.0;  // Smoothed RSSI value for distance approximation
static float alpha = 0.2;          // Low-pass filter coefficient

// RSSI timer callback function
void rssi_timer_callback(TimerHandle_t xTimer) {
    // Read RSSI of the connected device
    ESP_ERROR_CHECK(esp_ble_gap_read_rssi(connected_device_addr));
}

void start_rssi_timer() {
    // Start the RSSI polling timer
    if (rssi_timer == NULL) {
        ESP_LOGI(TAG, "Starting RSSI timer\nFiltered_RSSI,Raw_RSSI");
        rssi_timer = xTimerCreate("RSSI_Timer", pdMS_TO_TICKS(READ_RSSI_TIMEOUT_MS), pdTRUE, (void*)0, rssi_timer_callback);  // 2-second interval
        xTimerStart(rssi_timer, 0);
    } else {
        ESP_LOGW(TAG, "RSSI timer already exists, resetting");
        xTimerReset(rssi_timer, 0);  // Reset timer if it already exists
    }
}

void update_rssi(int rssi) {
    if (smoothed_rssi == 0.0) {
        smoothed_rssi = rssi;
    } else {
        smoothed_rssi = rssi * alpha + smoothed_rssi * (1.0 - alpha);  // Low-pass filter
    }
    ESP_LOGI(TAG, "%.2f,%d", smoothed_rssi, rssi);
}

void stop_rssi_timer() {
    // Stop the RSSI polling timer
    if (rssi_timer != NULL) {
        smoothed_rssi = 0.0;  // Reset smoothed RSSI value

        ESP_LOGI(TAG, "Stopping RSSI timer");
        xTimerStop(rssi_timer, 0);
    }
}

// GAP event handler for managing advertising
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Failed to start advertising");
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising stopped");
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            // Authentication complete event
            if (param->ble_security.auth_cmpl.success == true) {
                ESP_LOGI(TAG, "Authentication successful");
                // Start the RSSI polling timer
                start_rssi_timer();
            } else {
                ESP_LOGE(TAG, "Authentication failed");
            }
            break;
            
        case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
            if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                int rssi = param->read_rssi_cmpl.rssi;
                update_rssi(rssi);
            } else {
                ESP_LOGE(TAG, "Failed to read RSSI");
            }
            break;

        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set");
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan response data set");
            break;

        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            ESP_LOGI(TAG, "Set pkt length complete");
            break;

        case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
            ESP_LOGI(TAG, "PHY update complete");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection parameters updated");
            break;

        case ESP_GAP_BLE_KEY_EVT:
            ESP_LOGI(TAG, "Key event for peer device keys");
            break;

        default:
            ESP_LOGW(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

// GATT server event handler
void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            // The server is started, register the service
            ESP_LOGI(TAG, "GATT server registered, creating HID service");

            // Create the HID service
            esp_gatt_srvc_id_t hid_service_id = {
                .is_primary = true,
                .id.inst_id = 0x00,
                .id.uuid.len = ESP_UUID_LEN_16,
                .id.uuid.uuid.uuid16 = ESP_GATT_UUID_HID_SVC,
            };
            ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &hid_service_id, 8));
            
            // This will trigger a ESP_GATTS_CREATE_EVT event
            break;

        case ESP_GATTS_CREATE_EVT:
            // The service is created during the server callback, add characteristics
            ESP_LOGI(TAG, "Service created");

            hid_service_handle = param->create.service_handle;

            ESP_LOGI(TAG, "Adding HID characteristics");

            // Add HID Information characteristic
            // This is the bare mnimum characteristic required for the service
            // to be recognized as a human-connectable device
            esp_bt_uuid_t char_id = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = ESP_GATT_UUID_HID_INFORMATION
            };

            esp_attr_value_t hid_info_attr = {
                .attr_max_len = sizeof(hid_information),
                .attr_len = sizeof(hid_information),
                .attr_value = (uint8_t *)hid_information
            };
            ESP_ERROR_CHECK(esp_ble_gatts_add_char(hid_service_handle, &char_id, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, &hid_info_attr, NULL));

            // Start the HID service
            ESP_LOGI(TAG, "Starting HID service");
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(hid_service_handle));

            ESP_LOGI(TAG, "Starting advertising");
            // Configure advertising data
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

            // Configure scan response data
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));
            
            // Start advertising
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, connection ID: %d", param->connect.conn_id);

            // Store the connected device's address for RSSI tracking
            memcpy(connected_device_addr, param->connect.remote_bda, sizeof(esp_bd_addr_t));

            // Enable encryption with MITM protection for this connection
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            break;

        case ESP_GATTS_READ_EVT:
            uint16_t handle = param->read.handle;
            ESP_LOGI(TAG, "Read event, handle: %d", param->read.handle);

            // Respond to HID Information read requests
            if (handle == hid_info_handle) {
                ESP_LOGI(TAG, "HID Information requested, sending response");
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.len = sizeof(hid_information);
                memcpy(rsp.attr_value.value, hid_information, sizeof(hid_information));
                
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                            ESP_GATT_OK, &rsp);
            } else {
                ESP_LOGW(TAG, "Unhandled read request, handle: %d", handle);
            }

            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "Write event, handle: %d", param->write.handle);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected");
            stop_rssi_timer();
            
            // Clear the stored address
            memset(connected_device_addr, 0, sizeof(esp_bd_addr_t));

            // Restart advertising
            ESP_LOGI(TAG, "Restarting advertising");
            esp_ble_gap_start_advertising(&adv_params);  // Restart advertising on disconnection
            break;

        // Events during configuration
        case ESP_GATTS_ADD_CHAR_EVT:
            esp_gatt_status_t status = param->add_char.status;
            ESP_LOGI(TAG, "Characteristic added, status: %d, attr_handle: %d", param->add_char.status, param->add_char.attr_handle);
            if (status == ESP_GATT_OK) {
                hid_info_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Characteristic added successfully, handle stored");
            } else {
                ESP_LOGE(TAG, "Failed to add characteristic");
            }
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started, status: %d", param->start.status);
            break;

        // Events during connection
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "Set MTU complete, size: %d", param->mtu.mtu);
            break;

        case ESP_GATTS_CONGEST_EVT:
            ESP_LOGI(TAG, "Congestion event");
            break;

        case ESP_GATTS_RESPONSE_EVT:
            ESP_LOGI(TAG, "Response event");
            break;

        default:
            ESP_LOGW(TAG, "Unhandled GATT event: %d", event);
            break;
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize BLE
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Register GAP layer
    // Callback and device name for advertising
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gap_set_device_name(DEVICE_NAME);

    // Register GATT callback and application profile to receive GATT events
    esp_ble_gatts_register_callback(gatt_event_handler);

    // Set Application ID (can be 0 or any unique value)
    esp_ble_gatts_app_register(0);  

    // Setup secure connection mode with MITM protection
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, 
                                                   &auth_req, sizeof(auth_req)));

    // Set the maximum key size for encryption
    uint8_t key_size = 16;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, 
                                                   &key_size, sizeof(key_size)));

    // Setup IO capability
    uint8_t iocap = ESP_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, 
                                                   &iocap, sizeof(iocap)));

    // Set the initial key for encryption
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 
                                   sizeof(uint8_t));
}
