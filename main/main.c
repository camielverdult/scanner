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

// Define a 16-bit GATT Service UUID
static uint16_t service_uuid = 0x00FF;
static uint16_t gatt_service_handle = 0;

// Advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = (uint8_t*)&service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
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

// Store the address of the connected device
static esp_bd_addr_t connected_device_addr = {0};

// Timer to read RSSI periodically
static TimerHandle_t rssi_timer;

// RSSI read interval in milliseconds
#define READ_RSSI_TIMEOUT_MS 250

static float smoothed_rssi = 0.0;  // Smoothed RSSI value for distance approximation
static float alpha = 0.2;          // Low-pass filter coefficient

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
            ESP_LOGI(TAG, "Authentication completed, success: %d", param->ble_security.auth_cmpl.success);
            break;

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            ESP_LOGI(TAG, "Passkey requested");
            esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 123456);  // Example passkey
            break;

        case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
            if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                int rssi = param->read_rssi_cmpl.rssi;

                if (smoothed_rssi == 0.0) {
                    smoothed_rssi = rssi;
                } else {
                    smoothed_rssi = rssi * alpha + smoothed_rssi * (1.0 - alpha);  // Low-pass filter
                }
                ESP_LOGI(TAG, "Smoothed RSSI: %.2f dBm (Raw: %d dBm)", smoothed_rssi, rssi);
            } else {
                ESP_LOGE(TAG, "Failed to read RSSI");
            }
            break;

        default:
            break;
    }
}

// RSSI timer callback function
void rssi_timer_callback(TimerHandle_t xTimer) {
    // Read RSSI of the connected device
    esp_ble_gap_read_rssi(connected_device_addr);
}

// GATT server event handler
void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_gatt_srvc_id_t service_id;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered, creating service");

            // Create a primary service
            service_id.is_primary = true;
            service_id.id.inst_id = 0x00;
            service_id.id.uuid.len = ESP_UUID_LEN_16;
            service_id.id.uuid.uuid.uuid16 = service_uuid;  // Set service UUID

            // Create the service and allocate space for attributes
            esp_ble_gatts_create_service(gatts_if, &service_id, 4);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created, starting service and advertising");
            gatt_service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(gatt_service_handle);

            // Configure advertising data and start advertising
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, connection ID: %d", param->connect.conn_id);

            // Store the connected device's address for RSSI tracking
            memcpy(connected_device_addr, param->connect.remote_bda, sizeof(esp_bd_addr_t));

            // Start the RSSI polling timer
            if (rssi_timer == NULL) {
                rssi_timer = xTimerCreate("RSSI_Timer", pdMS_TO_TICKS(READ_RSSI_TIMEOUT_MS), pdTRUE, (void*)0, rssi_timer_callback);  // 2-second interval
                xTimerStart(rssi_timer, 0);
            } else {
                xTimerReset(rssi_timer, 0);  // Reset timer if it already exists
            }

            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, restarting advertising...");
            
            // Clear the stored address
            memset(connected_device_addr, 0, sizeof(esp_bd_addr_t));

            // Stop the RSSI polling timer
            if (rssi_timer != NULL) {
                xTimerStop(rssi_timer, 0);
            }

            smoothed_rssi = 0.0;  // Reset smoothed RSSI value

            esp_ble_gap_start_advertising(&adv_params);  // Restart advertising on disconnection
            break;

        default:
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

    // Set device name for BLE advertising
    esp_ble_gap_set_device_name(DEVICE_NAME);

    // Register GAP and GATT callbacks
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatt_event_handler);

    // Register application profile to receive GATT events
    esp_ble_gatts_app_register(0);  // Application ID can be 0 or any unique value

    // Set authentication and bonding parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  // Enable bonding with MITM protection
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
}
