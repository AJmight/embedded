#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// === CONFIGURATION ===
static const char *TAG = "SAFE_BT";

// Hardware Pins
#define TRIG_GPIO      GPIO_NUM_2
#define ECHO_GPIO      GPIO_NUM_4  
#define BUZZER_GPIO    GPIO_NUM_19
#define LED_GREEN      GPIO_NUM_21
#define LED_ORANGE     GPIO_NUM_22  
#define LED_RED        GPIO_NUM_23

// Detection Parameters
#define GREEN_ZONE     300.0f
#define ORANGE_ZONE    150.0f  
#define RED_ZONE       50.0f
#define ALARM_TIME     3

// System State
typedef enum {
    STATE_DISARMED = 0,
    STATE_ARMED = 1,
    STATE_ALARM = 2
} system_state_t;

system_state_t system_state = STATE_DISARMED;
float current_distance = 0.0f;
const char* current_zone = "OFFLINE";

// === BLUETOOTH CONFIGURATION ===
#define DEVICE_NAME             "ESP32_ALARM_SYSTEM"
#define GATTS_SERVICE_UUID      0xFFE0
#define GATTS_CHAR_UUID         0xFFE1

// BLE State
static bool connected_flag = false;
static uint16_t conn_id = 0;
static esp_gatt_if_t gatts_if;
static uint16_t char_handle;

// === BLUETOOTH MESSAGING ===
void send_bt_message(const char* message_type, const char* data) {
    if (!connected_flag) return;

    char full_message[64];
    int len = snprintf(full_message, sizeof(full_message), "BT_%s:%s", message_type, data);
    
    // Send as notification to connected device
    esp_ble_gatts_send_indicate(gatts_if, conn_id, char_handle, len, (uint8_t*)full_message, false);
    ESP_LOGI(TAG, "BT Sent: %s", full_message);
}

// === HARDWARE FUNCTIONS (Same as before) ===
float measure_distance_cm() {
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);
    
    int64_t timeout = esp_timer_get_time() + 10000;
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    int64_t start_time = esp_timer_get_time();
    
    timeout = esp_timer_get_time() + 25000;
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    int64_t end_time = esp_timer_get_time();
    
    return (float)(end_time - start_time) / 58.0f;
}

void update_leds(const char* zone) {
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_ORANGE, 0);
    gpio_set_level(LED_RED, 0);
    
    if (strcmp(zone, "GREEN") == 0) {
        gpio_set_level(LED_GREEN, 1);
    } else if (strcmp(zone, "ORANGE") == 0) {
        gpio_set_level(LED_ORANGE, 1);
    } else if (strcmp(zone, "RED") == 0) {
        gpio_set_level(LED_RED, 1);
    }
}

void buzzer_beep(int duration_ms) {
    gpio_set_level(BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(BUZZER_GPIO, 0);
}

void buzzer_alarm() {
    for(int i = 0; i < 2; i++) {
        buzzer_beep(200);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// === COMMAND PROCESSING ===
void process_command(const char* command) {
    ESP_LOGI(TAG, "Processing: %s", command);
    
    if (strstr(command, "ARM")) {
        system_state = STATE_ARMED;
        buzzer_beep(100);
        current_zone = "GREEN";
        update_leds(current_zone);
        send_bt_message("STATUS", "SAFE");
    } 
    else if (strstr(command, "DISARM")) {
        system_state = STATE_DISARMED;
        gpio_set_level(BUZZER_GPIO, 0);
        current_zone = "GREEN";
        update_leds(current_zone);
        send_bt_message("ALERT", "System DISARMED");
    } 
    else if (strstr(command, "POWER_OFF")) {
        system_state = STATE_DISARMED;
        current_zone = "OFFLINE";
        gpio_set_level(BUZZER_GPIO, 0);
        update_leds(current_zone);
        send_bt_message("STATUS", "OFFLINE");
    } 
    else if (strstr(command, "POWER_ON")) {
        system_state = STATE_ARMED;
        current_zone = "GREEN";
        update_leds(current_zone);
        send_bt_message("STATUS", "SAFE");
    }
    else {
        send_bt_message("ERROR", "Unknown command");
    }
}

// === DETECTION TASK ===
void detection_task(void *pvParameter) {
    int alarm_counter = 0;
    const char* last_zone = "GREEN";
    
    while (1) {
        if (system_state == STATE_ARMED || system_state == STATE_ALARM) {
            float distance = measure_distance_cm();
            current_distance = distance;
            
            if (distance > GREEN_ZONE || distance < 0) {
                current_zone = "GREEN";
            } else if (distance > ORANGE_ZONE) {
                current_zone = "ORANGE";
            } else if (distance > RED_ZONE) {
                current_zone = "RED";
            } else {
                current_zone = "ALARM";
            }
            
            if (strcmp(current_zone, last_zone) != 0) {
                char zone_data[32];
                snprintf(zone_data, sizeof(zone_data), "%s:%.1f", current_zone, distance);
                send_bt_message("ZONE_CHANGE", zone_data);
                last_zone = current_zone;
                
                if (system_state == STATE_ARMED) {
                    update_leds(current_zone);
                }
            }
            
            if (system_state == STATE_ARMED) {
                if (strcmp(current_zone, "RED") == 0 || strcmp(current_zone, "ALARM") == 0) {
                    alarm_counter++;
                    if (alarm_counter >= ALARM_TIME) {
                        system_state = STATE_ALARM;
                        buzzer_alarm();
                        send_bt_message("ALARM", "TRIGGERED");
                        alarm_counter = 0;
                    }
                } else {
                    alarm_counter = 0;
                }
            }
            else if (system_state == STATE_ALARM) {
                static int flash_counter = 0;
                gpio_set_level(LED_RED, (flash_counter % 2) == 0);
                gpio_set_level(BUZZER_GPIO, 1);
                flash_counter++;
            }
        } 
        else {
            alarm_counter = 0;
            gpio_set_level(BUZZER_GPIO, 0);
            if (strcmp(current_zone, "OFFLINE") != 0) {
                update_leds(current_zone);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// === BLUETOOTH CALLBACKS ===
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "Bluetooth registered");
            // Set device name
            esp_ble_gap_set_device_name(DEVICE_NAME);
            
            // Create service
            esp_bt_uuid_t srvc_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_SERVICE_UUID}
            };
            esp_ble_gatts_create_service(gatts_if, &srvc_uuid, 0, 3, 0);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created");
            // Start service
            esp_ble_gatts_start_service(param->create.service_handle);
            
            // Add characteristic
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID}
            };
            esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                                  ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "Characteristic added");
            char_handle = param->add_char.attr_handle;
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Device connected");
            connected_flag = true;
            conn_id = param->connect.conn_id;
            memcpy(&gatts_if, &gatts_if, sizeof(gatts_if));
            send_bt_message("CONNECT", "OK");
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Device disconnected");
            connected_flag = false;
            break;
            
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == char_handle) {
                char command[32] = {0};
                if (param->write.len < sizeof(command)) {
                    memcpy(command, param->write.value, param->write.len);
                    command[param->write.len] = '\0';
                    process_command(command);
                }
            }
            break;
            
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            // Start advertising
            esp_ble_adv_params_t adv_params = {
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            };
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully as: %s", DEVICE_NAME);
            }
            break;
            
        default:
            break;
    }
}

// === MAIN ===
void app_main() {
    // Initialize GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIG_GPIO) | (1ULL << BUZZER_GPIO) | 
                       (1ULL << LED_GREEN) | (1ULL << LED_ORANGE) | (1ULL << LED_RED),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << ECHO_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    gpio_set_level(BUZZER_GPIO, 0);
    update_leds("GREEN");
    
    ESP_LOGI(TAG, "🚀 Starting Bluetooth Safe System...");
    
    // === BLUETOOTH INITIALIZATION ===
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    
    // Register GATT application
    esp_ble_gatts_app_register(0);
    
    // Start detection task
    xTaskCreate(detection_task, "detection_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✅ System ready! Broadcasting as: %s", DEVICE_NAME);
}
