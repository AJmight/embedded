#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"    
#include "driver/uart.h"

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

system_state_t system_state = STATE_DISARMED;
float current_distance = 0.0f;
const char* current_zone = "GREEN";

// === BLUETOOTH SIMULATION (Serial) ===
void send_bt_message(const char* message_type, const char* data) {
    printf("BT_%s:%s\n", message_type, data);
}

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
    return (float)(end_time - start_time) / 58.0f;
}

// === LED INDICATORS ===
void update_leds(const char* zone) {
    // Turn all LEDs off first
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_ORANGE, 0);
    gpio_set_level(LED_RED, 0);
    
    // Turn on appropriate LED
    if (strcmp(zone, "GREEN") == 0) {
        gpio_set_level(LED_GREEN, 1);
    } else if (strcmp(zone, "ORANGE") == 0) {
        gpio_set_level(LED_ORANGE, 1);
    } else if (strcmp(zone, "RED") == 0) {
        gpio_set_level(LED_RED, 1);
    }
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

void buzzer_zone_alert(const char* zone) {
    if (strcmp(zone, "ORANGE") == 0) {
        buzzer_beep(100); // Short beep for orange zone
    } else if (strcmp(zone, "RED") == 0) {
        buzzer_beep(500); // Longer beep for red zone
    }
}

// === COMMAND PROCESSING ===
void process_command(const char* command) {
    ESP_LOGI(TAG, "Processing command: %s", command);
    
    if (strstr(command, "ARM")) {
        system_state = STATE_ARMED;
        buzzer_beep(100);
        update_leds(current_zone);
        send_bt_message("ALERT", "System ARMED");
        ESP_LOGI(TAG, "System armed via Bluetooth");
    } 
    else if (strstr(command, "DISARM")) {
        system_state = STATE_DISARMED;
        gpio_set_level(BUZZER_GPIO, 0);
        update_leds("GREEN");
        send_bt_message("ALERT", "System DISARMED");
        ESP_LOGI(TAG, "System disarmed via Bluetooth");
    } 
    else if (strstr(command, "STATUS")) {
        char status_data[64];
        snprintf(status_data, sizeof(status_data), "%.1f:%s:%d", 
                current_distance, current_zone, system_state);
        send_bt_message("STATUS", status_data);
    } 
    else if (strstr(command, "GET_ZONE")) {
        send_bt_message("ZONE", current_zone);
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
        float distance = measure_distance_cm();
        current_distance = distance;
        
        // Determine zone
        if (distance > GREEN_ZONE) {
            current_zone = "GREEN";
        } else if (distance > ORANGE_ZONE) {
            current_zone = "ORANGE";
        } else if (distance > 0) {
            current_zone = "RED";
        } else {
            current_zone = "UNKNOWN";
        }
        
        // Zone change detection
        if (strcmp(current_zone, last_zone) != 0) {
            ESP_LOGI(TAG, "Zone changed: %s -> %s (%.1f cm)", last_zone, current_zone, distance);
            
            // Send zone change notification
            char zone_data[32];
            snprintf(zone_data, sizeof(zone_data), "%s:%.1f", current_zone, distance);
            send_bt_message("ZONE_CHANGE", zone_data);
            
            last_zone = current_zone;
            
            if (system_state == STATE_ARMED) {
                update_leds(current_zone);
                buzzer_zone_alert(current_zone);
            }
        }
        
        // Alarm logic
        if (system_state == STATE_ARMED && distance > 0) {
            if (strcmp(current_zone, "RED") == 0) {
                alarm_counter++;
                ESP_LOGI(TAG, "🚨 Person in RED zone: %.2f cm (%d/%d)", 
                        distance, alarm_counter, ALARM_TIME);
                
                // Send alert
                char alert_data[32];
                snprintf(alert_data, sizeof(alert_data), "RED:%.1f:%d", distance, alarm_counter);
                send_bt_message("ALERT", alert_data);
                
                if (alarm_counter >= ALARM_TIME) {
                    system_state = STATE_ALARM;
                    buzzer_alarm();
                    send_bt_message("ALARM", "TRIGGERED");
                    ESP_LOGE(TAG, "ALARM TRIGGERED! Person detected for %d seconds", ALARM_TIME);
                    alarm_counter = 0;
                }
            } else {
                alarm_counter = 0; // Reset if not in red zone
            }
        } 
        else if (system_state == STATE_ALARM) {
            // Keep alarming until disarmed
            gpio_set_level(BUZZER_GPIO, 1);
        } 
        else {
            // System disarmed
            alarm_counter = 0;
            gpio_set_level(BUZZER_GPIO, 0);
            update_leds(current_zone);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// === SERIAL COMMAND TASK (Bluetooth Simulation) ===
void serial_task(void *pvParameter) {
    char buffer[64];
    int index = 0;
    
    ESP_LOGI(TAG, "Serial command task started. Ready for commands.");
    
    while (1) {
        // Read serial input (simulates Bluetooth commands)
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                if (index > 0) {
                    buffer[index] = '\0';
                    process_command(buffer);
                    index = 0;
                }
            } else if (index < sizeof(buffer) - 1) {
                buffer[index++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// === MAIN ===
void app_main() {
    // Initialize GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIG_GPIO) | (1ULL << BUZZER_GPIO) | 
                       (1ULL << LED_GREEN) | (1ULL << LED_ORANGE) | (1ULL << LED_RED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << ECHO_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    // Initialize all outputs to LOW
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_ORANGE, 0);
    gpio_set_level(LED_RED, 0);
    
    ESP_LOGI(TAG, "🚀 Starting Bluetooth Safe System...");
    ESP_LOGI(TAG, "🎯 Zone distances: GREEN>%.0fcm, ORANGE>%.0fcm, RED<%.0fcm", 
            GREEN_ZONE, ORANGE_ZONE, RED_ZONE);
    ESP_LOGI(TAG, "📱 Send commands via Serial Monitor:");
    ESP_LOGI(TAG, "   - ARM: Arm the system");
    ESP_LOGI(TAG, "   - DISARM: Disarm the system");  
    ESP_LOGI(TAG, "   - STATUS: Get current status");
    ESP_LOGI(TAG, "   - GET_ZONE: Get current zone");
    
    // Start green LED to indicate system ready
    gpio_set_level(LED_GREEN, 1);
    
    // Start tasks
    xTaskCreate(detection_task, "detection_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_task, "serial_task", 2048, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "✅ System ready! Monitoring for intrusions...");
}
