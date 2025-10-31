#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

static const char *TAG = "SECURE_VAULT";

// WiFi Configuration
#define WIFI_SSID       "SecureVault_AP"
#define WIFI_PASS       "12345678"
#define PORT            3333
#define STREAM_INTERVAL_MS 500

// System State
typedef enum {
    STATE_DISARMED,
    STATE_ARMING, 
    STATE_ARMED,
    STATE_ALERT
} SystemState;

// Global Variables
SystemState current_state = STATE_DISARMED;
float distance_cm = 100.0f;
volatile int g_client_sock = -1;

// WiFi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "✅ WiFi AP Ready! Connect to: %s", WIFI_SSID);
        ESP_LOGI(TAG, "📱 IP: 192.168.4.1, Port: %d", PORT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "📱 Mobile App Connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "📱 Mobile App Disconnected");
        g_client_sock = -1;
    }
}

void wifi_init_softap(void) {
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // WiFi initialization
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    // WiFi configuration
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = 6
        },
    };
    
    // Set country code (important for stability)
    esp_wifi_set_country_code("US", true);
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "📶 WiFi AP: %s", WIFI_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", WIFI_PASS);
}

// Send status to mobile app
static void send_to_mobile(int sock, const char* message) {
    if (sock < 0) return;
    
    if (send(sock, message, strlen(message), 0) < 0) {
        ESP_LOGE(TAG, "Failed to send to mobile");
        close(sock);
        g_client_sock = -1;
    }
}

static void send_status_update(int sock) {
    char status_msg[128];
    const char* state_str = "DISARMED";
    
    if (current_state == STATE_ARMED) state_str = "ARMED";
    else if (current_state == STATE_ARMING) state_str = "ARMING";
    else if (current_state == STATE_ALERT) state_str = "ALERT";

    snprintf(status_msg, sizeof(status_msg), 
             "STATE:%s,DISTANCE:%.1fcm\n", state_str, distance_cm);
    
    send_to_mobile(sock, status_msg);
}

// Handle mobile app commands
void mobile_app_handler(void *pvParameters) {
    int sock = (int)pvParameters;
    g_client_sock = sock;
    char rx_buffer[128];
    
    ESP_LOGI(TAG, "📱 Mobile app session started");
    send_status_update(sock); // Send initial status

    TickType_t last_stream_time = xTaskGetTickCount();

    while (1) {
        // Check for commands from mobile app
        struct timeval timeout = { .tv_sec = 0, .tv_usec = 100000 }; // 100ms
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

        if (len > 0) {
            rx_buffer[len] = 0;
            // Clean up the command
            if (rx_buffer[len-1] == '\n') rx_buffer[len-1] = 0;
            if (rx_buffer[len-1] == '\r') rx_buffer[len-1] = 0;
            
            ESP_LOGI(TAG, "📱 Mobile command: %s", rx_buffer);

            // Process ARM command
            if (strstr(rx_buffer, "ARM") != NULL) {
                if (current_state != STATE_ARMED) {
                    current_state = STATE_ARMING;
                    send_status_update(sock);
                    ESP_LOGI(TAG, "🔄 Arming system...");
                    
                    // 3 second arming delay
                    for(int i = 3; i > 0; i--) {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        char countdown[64];
                        snprintf(countdown, sizeof(countdown), "COUNTDOWN:%d\n", i);
                        send_to_mobile(sock, countdown);
                    }
                    
                    current_state = STATE_ARMED;
                    send_status_update(sock);
                    ESP_LOGI(TAG, "✅ System ARMED");
                }
            } 
            // Process DISARM command
            else if (strstr(rx_buffer, "DISARM") != NULL) {
                current_state = STATE_DISARMED;
                send_status_update(sock);
                ESP_LOGI(TAG, "❌ System DISARMED");
            }
            else {
                send_to_mobile(sock, "ERROR:Unknown command\n");
            }
        } else if (len == 0) {
            ESP_LOGI(TAG, "📱 Mobile app disconnected");
            break;
        }
        
        // Stream data when armed (every 500ms)
        if ((current_state == STATE_ARMED || current_state == STATE_ALERT) && 
            (xTaskGetTickCount() - last_stream_time > pdMS_TO_TICKS(STREAM_INTERVAL_MS))) {
            send_status_update(sock);
            last_stream_time = xTaskGetTickCount();
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Cleanup
    close(sock);
    g_client_sock = -1;
    ESP_LOGI(TAG, "📱 Mobile app session ended");
    vTaskDelete(NULL);
}

// TCP Server for mobile app connections
void tcp_server_task(void *pvParameters) {
    int listen_sock = -1;
    
    // Create server socket
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "❌ Failed to create socket");
        goto CLEAN_UP;
    }
    
    // Allow socket reuse
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind and listen
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "❌ Failed to bind socket");
        goto CLEAN_UP;
    }
    
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "❌ Failed to listen");
        goto CLEAN_UP;
    }

    ESP_LOGI(TAG, "✅ TCP Server listening on port %d", PORT);

    while (1) {
        // Accept new mobile app connections
        if (g_client_sock == -1) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            
            int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
            if (client_sock >= 0) {
                ESP_LOGI(TAG, "📱 New mobile app connected");
                xTaskCreate(mobile_app_handler, "mobile_handler", 4096, (void*)client_sock, 5, NULL);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

CLEAN_UP:
    if (listen_sock != -1) close(listen_sock);
    vTaskDelete(NULL);
}

// Simulate distance sensor readings
void sensor_simulator_task(void *pvParameters) {
    ESP_LOGI(TAG, "🔍 Starting sensor simulator");
    
    while(1) {
        if (current_state == STATE_ARMED) {
            // Simulate realistic distance changes
            float change = ((esp_random() % 10) - 5) * 0.3f; // Small random changes
            distance_cm += change;
            
            // Keep distance in realistic range
            if (distance_cm < 5.0f) distance_cm = 5.0f;
            if (distance_cm > 400.0f) distance_cm = 400.0f;
            
            // Trigger alert if object too close
            if (distance_cm < 30.0f && current_state != STATE_ALERT) {
                ESP_LOGW(TAG, "🚨 ALERT! Object at %.1f cm", distance_cm);
                current_state = STATE_ALERT;
                
                // Send immediate alert to mobile app
                if (g_client_sock != -1) {
                    send_status_update(g_client_sock);
                }
            } 
            // Clear alert if object moves away
            else if (distance_cm >= 50.0f && current_state == STATE_ALERT) {
                ESP_LOGI(TAG, "✅ Alert cleared");
                current_state = STATE_ARMED;
                
                // Send status update to mobile app
                if (g_client_sock != -1) {
                    send_status_update(g_client_sock);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting SecureVault System...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start WiFi Access Point
    wifi_init_softap();

    // Start TCP server for mobile app
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    
    // Start sensor simulator
    xTaskCreate(sensor_simulator_task, "sensor_sim", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "✅ SecureVault System Ready!");
    ESP_LOGI(TAG, "📶 Connect to WiFi: %s", WIFI_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", WIFI_PASS);
    ESP_LOGI(TAG, "📱 App will auto-connect to 192.168.4.1:%d", PORT);
}
