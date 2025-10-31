#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "SECURE_VAULT";

// === UUID CONSTANTS ===
static const uint16_t primary_service_uuid = 0x2800;
static const uint16_t character_declaration_uuid = 0x2803;
static const uint16_t character_client_config_uuid = 0x2902;
static const uint8_t char_prop_write = 0x08;        // Write property
static const uint8_t char_prop_notify = 0x10;       // Notify property

// === STATE MANAGEMENT ===
typedef enum {
    STATE_DISARMED,
    STATE_ARMING,
    STATE_ARMED,
    STATE_ALERT
} SystemState;

SystemState current_state = STATE_DISARMED;
uint16_t gatt_conn_id = 0xFFFF;
uint16_t tx_handle_table[1];
float distanceCm = 100.0f;  // Start at 100cm

#define PROFILE_NUM      1
#define PROFILE_APP_ID   0
#define SVC_INST_ID      0

// === UUID CONFIGURATION (MATCHES ANDROID APP) ===
#define SECURE_VAULT_SERVICE_UUID 0x4AC1
#define RX_CHAR_UUID              0x4AC10001  // App writes ARM/DISARM here
#define TX_CHAR_UUID              0x4AC10002  // App subscribes for notifications

// === ATTRIBUTE TABLE ===
#define GATTS_NUM_HANDLE     6  // Exactly 6 attributes needed
uint16_t secure_vault_handle_table[GATTS_NUM_HANDLE];

// === ADVERTISING CONFIG ===
static uint8_t secure_vault_service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 
    0x00, 0x10, 0x00, 0x00, 0xC1, 0x4A, 0x00, 0x00  // 4ac10000-0000-1000-8000-00805f9b34fb
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    .p_service_uuid      = secure_vault_service_uuid,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// === GATT SERVICE DEFINITION ===
static const esp_gatts_attr_db_t gatt_db[GATTS_NUM_HANDLE] = {
    // Service Declaration
    [0] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t *)&primary_service_uuid,
            .perm = ESP_GATT_PERM_READ,
            .max_length = sizeof(uint16_t),
            .value_length = sizeof(uint16_t),
            .value = (uint8_t *)&SECURE_VAULT_SERVICE_UUID,
        },
    },

    // RX Characteristic Declaration (Write)
    [1] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t *)&character_declaration_uuid,
            .perm = ESP_GATT_PERM_READ,
            .max_length = 1,
            .value_length = 1,
            .value = (uint8_t *)&char_prop_write,
        },
    },

    // RX Characteristic Value
    [2] = {
        .attr_control = {.auto_rsp = ESP_GATT_RSP_BY_APP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_32,
            .uuid_p = (uint8_t *)&(uint32_t){RX_CHAR_UUID},
            .perm = ESP_GATT_PERM_WRITE,
            .max_length = 64,
            .value_length = 0,
            .value = NULL,
        },
    },

    // TX Characteristic Declaration (Notify)
    [3] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t *)&character_declaration_uuid,
            .perm = ESP_GATT_PERM_READ,
            .max_length = 1,
            .value_length = 1,
            .value = (uint8_t *)&char_prop_notify,
        },
    },

    // TX Characteristic Value
    [4] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_32,
            .uuid_p = (uint8_t *)&(uint32_t){TX_CHAR_UUID},
            .perm = 0,
            .max_length = 64,
            .value_length = 0,
            .value = NULL,
        },
    },

    // CCCD Descriptor (CRITICAL FOR NOTIFICATIONS)
    [5] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t *)&character_client_config_uuid,
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = 2,
            .value_length = 2,
            .value = (uint8_t *)&(uint16_t){0x0000},  // Notifications disabled initially
        },
    },
};

// === PROFILE STRUCTURE ===
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct gatts_profile_inst secure_vault_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gatts_cb = NULL,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// === FORWARD DECLARATIONS ===
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void update_leds(const char *color);

// === NOTIFICATION SENDING ===
void send_status_update(SystemState state, float distance) {
    if (gatt_conn_id == 0xFFFF || tx_handle_table[0] == 0) {
        ESP_LOGW(TAG, "Not connected or TX handle invalid");
        return;
    }

    char message[64];
    int len = 0;

    switch(state) {
        case STATE_ARMED: 
            len = snprintf(message, sizeof(message), "SYSTEM ARMED");
            update_leds("RED");
            break;
        case STATE_DISARMED: 
            len = snprintf(message, sizeof(message), "SYSTEM DISARMED");
            update_leds("GREEN");
            break;
        case STATE_ARMING: 
            len = snprintf(message, sizeof(message), "SYSTEM ARMING");
            update_leds("YELLOW");
            break;
        case STATE_ALERT:
            len = snprintf(message, sizeof(message), "BT_ZONE_CHANGE:A1:%.2f", distance);
            break;
        default:
            len = snprintf(message, sizeof(message), "SYSTEM UNKNOWN");
            break;
    }

    esp_err_t ret = esp_ble_gatts_send_notification(
        secure_vault_profile_tab[PROFILE_APP_ID].gatts_if,
        gatt_conn_id,
        tx_handle_table[0],
        len,
        (uint8_t *)message
    );
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Notification sent: %s", message);
    } else {
        ESP_LOGE(TAG, "Failed to send notification: 0x%x", ret);
    }
}

// === GAP EVENT HANDLER ===
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertisement data set complete");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;
            
        default:
            break;
    }
}

// === GATTS EVENT HANDLER ===
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (gatts_if != secure_vault_profile_tab[PROFILE_APP_ID].gatts_if) {
        return;
    }

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATTS registered successfully");
            esp_ble_gap_set_device_name("SecureVault_Alarm");
            esp_ble_gap_config_adv_data(&adv_data);
            break;

        case ESP_GATTS_CREATE_ATTR_TAB_EVT:
            if (param->create_attr_tab.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Attribute table created successfully");
                memcpy(secure_vault_handle_table, param->create_attr_tab.handle, sizeof(secure_vault_handle_table));
                tx_handle_table[0] = secure_vault_handle_table[4]; // TX characteristic value handle
                esp_ble_gatts_start_service(secure_vault_handle_table[0]);
            } else {
                ESP_LOGE(TAG, "Attribute table creation failed: 0x%x", param->create_attr_tab.status);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            gatt_conn_id = param->connect.conn_id;
            ESP_LOGI(TAG, "Device connected - conn_id: 0x%x", gatt_conn_id);
            esp_ble_gap_stop_advertising();
            send_status_update(current_state, distanceCm);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Device disconnected - restarting advertising");
            gatt_conn_id = 0xFFFF;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "Write event - handle: 0x%x, len: %d", param->write.handle, param->write.len);

            if (param->write.handle == secure_vault_handle_table[2]) {
                // Command received on RX characteristic
                char command[32] = {0};
                memcpy(command, param->write.value, param->write.len);
                command[param->write.len] = '\0';
                
                ESP_LOGI(TAG, "Command received: %s", command);

                if (strstr(command, "ARM") != NULL) {
                    current_state = STATE_ARMING;
                    send_status_update(STATE_ARMING, 0.0f);
                    
                    // Simulate 3 second arming process
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    
                    current_state = STATE_ARMED;
                    send_status_update(STATE_ARMED, 0.0f);
                    ESP_LOGI(TAG, "System ARMED");
                    
                } else if (strstr(command, "DISARM") != NULL) {
                    current_state = STATE_DISARMED;
                    send_status_update(STATE_DISARMED, 0.0f);
                    ESP_LOGI(TAG, "System DISARMED");
                }
            } 
            else if (param->write.handle == secure_vault_handle_table[5]) {
                // CCCD write - notifications enabled/disabled
                uint16_t cccd_value = (param->write.value[1] << 8) | param->write.value[0];
                if (cccd_value == 0x0001) {
                    ESP_LOGI(TAG, "Notifications ENABLED by client");
                } else {
                    ESP_LOGI(TAG, "Notifications DISABLED by client");
                }
            }

            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;

        default:
            break;
    }
}

// === DISTANCE SIMULATION TASK ===
void distance_simulator_task(void *pvParameters) {
    ESP_LOGI(TAG, "Distance simulator task started");
    
    // Wait for BLE to initialize
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    while(1) {
        if (current_state == STATE_ARMED || current_state == STATE_ALERT) {
            // Simulate realistic distance changes
            distanceCm += ((rand() % 21) - 10) * 0.5f; // -5 to +5 cm change
            
            // Keep distance in realistic range
            if (distanceCm < 10.0f) distanceCm = 10.0f;
            if (distanceCm > 200.0f) distanceCm = 200.0f;
            
            // Check for alert condition
            if (distanceCm < 30.0f && current_state == STATE_ARMED) {
                current_state = STATE_ALERT;
                ESP_LOGW(TAG, "🚨 ALERT! Intruder at %.1f cm", distanceCm);
            } else if (distanceCm >= 30.0f && current_state == STATE_ALERT) {
                current_state = STATE_ARMED;
                ESP_LOGI(TAG, "Alert cleared - distance: %.1f cm", distanceCm);
            }
            
            // Send distance update
            send_status_update(STATE_ALERT, distanceCm);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // Update every 2 seconds
    }
}

// === LED CONTROL (DUMMY FUNCTION - REPLACE WITH ACTUAL GPIO) ===
void update_leds(const char *color) {
    ESP_LOGI(TAG, "LEDs set to: %s", color);
    // Replace with actual GPIO control:
    // gpio_set_level(LED_RED, 0);
    // gpio_set_level(LED_GREEN, 0);
    // gpio_set_level(LED_BLUE, 0);
    // if (strcmp(color, "RED") == 0) gpio_set_level(LED_RED, 1);
    // else if (strcmp(color, "GREEN") == 0) gpio_set_level(LED_GREEN, 1);
    // else if (strcmp(color, "YELLOW") == 0) { gpio_set_level(LED_RED, 1); gpio_set_level(LED_GREEN, 1); }
}

// === MAIN APPLICATION ===
void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting SecureVault Alarm System...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release classic BT memory
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS callback register failed: 0x%x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP callback register failed: 0x%x", ret);
        return;
    }

    // Register application
    secure_vault_profile_tab[PROFILE_APP_ID].gatts_cb = gatts_event_handler;
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: 0x%x", ret);
        return;
    }

    // Set initial state
    current_state = STATE_DISARMED;
    update_leds("GREEN");

    // Start distance simulation
    xTaskCreate(distance_simulator_task, "distance_sim", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "✅ SecureVault Alarm System Ready!");
    ESP_LOGI(TAG, "📱 Connect with Android app - Device: 'SecureVault_Alarm'");
}
