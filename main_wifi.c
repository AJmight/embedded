#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// === CONFIGURATION ===
static const char *TAG = "SAFE_WIFI";

// WiFi Configuration
#define WIFI_SSID      "Wokwi-GUEST"
#define WIFI_PASS      ""

// Hardware Pins (5V tolerant)
#define TRIG_GPIO      GPIO_NUM_2
#define ECHO_GPIO      GPIO_NUM_4  
#define BUZZER_GPIO    GPIO_NUM_19

// Detection Parameters
#define GREEN_ZONE     250.0f  // > 2.5m
#define ORANGE_ZONE    100.0f  // 1m - 2.5m  
#define RED_ZONE       50.0f   // < 1m
#define ALARM_TIME     3       // seconds in red zone before alarm

// System State
typedef enum {
    STATE_DISARMED = 0,
    STATE_ARMED = 1,
    STATE_ALARM = 2
} system_state_t;

volatile system_state_t system_state = STATE_DISARMED;
EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// === ULTRASONIC SENSOR ===
float measure_distance_cm() {
    // Ensure clean state
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    
    // Send 10µs trigger pulse
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);
    
    // Wait for echo start
    int64_t timeout = esp_timer_get_time() + 10000;
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    int64_t start_time = esp_timer_get_time();
    
    // Wait for echo end
    timeout = esp_timer_get_time() + 25000;
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    int64_t end_time = esp_timer_get_time();
    
    // Calculate distance (time in µs / 58 = cm)
    int64_t pulse_duration = end_time - start_time;
    if (pulse_duration <= 0) return -1.0f;
    
    return (float)pulse_duration / 58.0f;
}

// === BUZZER CONTROL ===
void buzzer_beep(int duration_ms) {
    gpio_set_level(BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(BUZZER_GPIO, 0);
}

void buzzer_alarm() {
    for(int i = 0; i < 10; i++) {
        buzzer_beep(200);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// === WEB SERVER HANDLERS ===
static esp_err_t root_handler(httpd_req_t *req) {
    const char* status = system_state == STATE_DISARMED ? "DISARMED" : 
                        system_state == STATE_ARMED ? "ARMED" : "ALARM";
    
    char response[512];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "\r\n"
        "{\"status\":\"%s\",\"system\":\"WiFi Safe System\",\"message\":\"Use /arm, /disarm, /status\"}", 
        status);
    
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t arm_handler(httpd_req_t *req) {
    system_state = STATE_ARMED;
    buzzer_beep(100);
    
    char response[200];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "\r\n"
        "{\"message\":\"System ARMED\",\"status\":\"armed\"}");
    
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "System armed via web");
    return ESP_OK;
}

static esp_err_t disarm_handler(httpd_req_t *req) {
    system_state = STATE_DISARMED;
    gpio_set_level(BUZZER_GPIO, 0);
    
    char response[200];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "\r\n"
        "{\"message\":\"System DISARMED\",\"status\":\"disarmed\"}");
    
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "System disarmed via web");
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    float distance = measure_distance_cm();
    const char* zone = "UNKNOWN";
    
    if (distance > GREEN_ZONE) zone = "GREEN";
    else if (distance > ORANGE_ZONE) zone = "ORANGE"; 
    else if (distance > 0) zone = "RED";
    else zone = "ERROR";
    
    char response[300];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "\r\n"
        "{\"distance\":%.2f,\"zone\":\"%s\",\"state\":%d,\"alarm_threshold\":%d}",
        distance, zone, system_state, ALARM_TIME);
    
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// === URI HANDLERS ===
static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler,
    .user_ctx = NULL
};

static const httpd_uri_t arm = {
    .uri = "/arm",
    .method = HTTP_GET,
    .handler = arm_handler,
    .user_ctx = NULL
};

static const httpd_uri_t disarm = {
    .uri = "/disarm", 
    .method = HTTP_GET,
    .handler = disarm_handler,
    .user_ctx = NULL
};

static const httpd_uri_t status = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
};

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &arm);
        httpd_register_uri_handler(server, &disarm);
        httpd_register_uri_handler(server, &status);
        ESP_LOGI(TAG, "Web server started on port: %d", config.server_port);
        return server;
    }
    
    ESP_LOGE(TAG, "Failed to start web server");
    return NULL;
}

// === WIFI EVENT HANDLER ===
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying WiFi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
                                          WIFI_CONNECTED_BIT,
                                          pdFALSE, pdTRUE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi successfully!");
    }
}

// === DETECTION TASK ===
void detection_task(void *pvParameter) {
    int alarm_counter = 0;
    
    while (1) {
        float distance = measure_distance_cm();
        
        if (system_state == STATE_ARMED && distance > 0) {
            if (distance < RED_ZONE) {
                alarm_counter++;
                ESP_LOGI(TAG, "🚨 Person in RED zone: %.2f cm (Counter: %d/%d)", 
                        distance, alarm_counter, ALARM_TIME);
                
                if (alarm_counter >= ALARM_TIME) {
                    system_state = STATE_ALARM;
                    buzzer_alarm();
                    ESP_LOGE(TAG, "ALARM TRIGGERED! Person detected for %d seconds", ALARM_TIME);
                    alarm_counter = 0;
                }
            } else if (distance < ORANGE_ZONE) {
                ESP_LOGI(TAG, "🟠 Person in ORANGE zone: %.2f cm", distance);
                alarm_counter = 0; // Reset counter when person moves away
            } else {
                ESP_LOGI(TAG, "🟢 Person in GREEN zone: %.2f cm", distance);
                alarm_counter = 0;
            }
        } else if (system_state == STATE_ALARM) {
            // Keep alarming until disarmed
            gpio_set_level(BUZZER_GPIO, 1);
            ESP_LOGE(TAG, "ALARM ACTIVE - Waiting for disarm command");
        } else {
            // System disarmed or error state
            alarm_counter = 0;
            gpio_set_level(BUZZER_GPIO, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}

// === MAIN APPLICATION ===
void app_main() {
    ESP_LOGI(TAG, "🚀 Starting Safe Security System...");
    
    // Initialize GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIG_GPIO) | (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << ECHO_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    // Ensure buzzer starts OFF
    gpio_set_level(BUZZER_GPIO, 0);
    
    ESP_LOGI(TAG, "GPIO initialized successfully");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "NVS flash initialized");

    // Start WiFi
    wifi_init_sta();
    
    // Start web server
    httpd_handle_t server = start_webserver();
    if (server) {
        ESP_LOGI(TAG, "Web server started successfully");
    }

    // Start detection task
    xTaskCreate(detection_task, "detection_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Detection task started");

    ESP_LOGI(TAG, "✅ System initialization complete!");
    ESP_LOGI(TAG, "👉 Use web browser to control the system");
    ESP_LOGI(TAG, "👉 Endpoints: /arm, /disarm, /status");
}