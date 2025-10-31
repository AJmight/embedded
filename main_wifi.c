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

#define WIFI_SSID "SecureVault_AP"
#define WIFI_PASS "12345678"
#define PORT 3333

typedef enum {
    STATE_DISARMED,
    STATE_ARMING, 
    STATE_ARMED,
    STATE_ALERT
} SystemState;

SystemState current_state = STATE_DISARMED;
float distance_cm = 100.0f;
volatile int g_client_sock = -1;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP Started: %s", WIFI_SSID);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Mobile app connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Mobile app disconnected");
        g_client_sock = -1;
    }
}

void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi: %s, Password: %s", WIFI_SSID, WIFI_PASS);
}

void send_to_mobile(int sock, const char* message) {
    if (sock < 0) return;
    send(sock, message, strlen(message), 0);
}

void send_status(int sock) {
    char msg[128];
    const char* state_str = "DISARMED";
    if (current_state == STATE_ARMED) state_str = "ARMED";
    else if (current_state == STATE_ARMING) state_str = "ARMING";
    else if (current_state == STATE_ALERT) state_str = "ALERT";

    snprintf(msg, sizeof(msg), "STATE:%s,DISTANCE:%.1fcm\n", state_str, distance_cm);
    send_to_mobile(sock, msg);
}

void mobile_handler(void *pvParameters) {
    int sock = (int)pvParameters;
    g_client_sock = sock;
    char buffer[128];

    ESP_LOGI(TAG, "Mobile app session started");
    send_status(sock);

    while (1) {
        int len = recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (len > 0) {
            buffer[len] = 0;
            if (buffer[len-1] == '\n') buffer[len-1] = 0;

            ESP_LOGI(TAG, "Command: %s", buffer);

            if (strstr(buffer, "ARM")) {
                if (current_state != STATE_ARMED) {
                    current_state = STATE_ARMING;
                    send_status(sock);
                    
                    for(int i = 3; i > 0; i--) {
                        char countdown[32];
                        snprintf(countdown, sizeof(countdown), "COUNTDOWN:%d\n", i);
                        send_to_mobile(sock, countdown);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                    
                    current_state = STATE_ARMED;
                    send_status(sock);
                }
            } else if (strstr(buffer, "DISARM")) {
                current_state = STATE_DISARMED;
                send_status(sock);
            }
        } else if (len == 0) {
            break;
        }
        
        if (current_state == STATE_ARMED || current_state == STATE_ALERT) {
            send_status(sock);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    close(sock);
    g_client_sock = -1;
    vTaskDelete(NULL);
}

void tcp_server_task(void *pvParameters) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    listen(listen_sock, 1);

    ESP_LOGI(TAG, "TCP server on port %d", PORT);

    while (1) {
        if (g_client_sock == -1) {
            int client_sock = accept(listen_sock, NULL, NULL);
            if (client_sock >= 0) {
                xTaskCreate(mobile_handler, "mobile_handler", 4096, (void*)client_sock, 5, NULL);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensor_task(void *pvParameters) {
    while (1) {
        if (current_state == STATE_ARMED) {
            distance_cm += ((esp_random() % 10) - 5) * 0.5f;
            if (distance_cm < 10.0f) distance_cm = 10.0f;
            if (distance_cm > 300.0f) distance_cm = 300.0f;
            
            if (distance_cm < 30.0f && current_state != STATE_ALERT) {
                current_state = STATE_ALERT;
                if (g_client_sock != -1) send_status(g_client_sock);
            } else if (distance_cm >= 50.0f && current_state == STATE_ALERT) {
                current_state = STATE_ARMED;
                if (g_client_sock != -1) send_status(g_client_sock);
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    nvs_flash_init();
    wifi_init_softap();
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "System Ready!");
}
