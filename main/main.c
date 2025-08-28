// Core ESP32 includes
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

// WiFi and HTTP server includes for same-network communication
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "cJSON.h"

// BLE includes for target detection
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "host/ble_store.h" 
#include "host/ble_hs_adv.h"

// Helper for address-to-string if not provided by NimBLE
static char *ble_addr_to_str(const ble_addr_t *addr, char *str) {
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            addr->val[5], addr->val[4], addr->val[3],
            addr->val[2], addr->val[1], addr->val[0]);
    return str;
}

static const char *TAG = "ble_scan";

// Motor pins for ESP32-C6 (adjust to your wiring)
#define LEFT_LPWM   GPIO_NUM_6   // PWM pin for left forward
#define LEFT_RPWM   GPIO_NUM_7   // PWM pin for left reverse
#define RIGHT_LPWM  GPIO_NUM_5   // PWM pin for right forward
#define RIGHT_RPWM  GPIO_NUM_4   // PWM pin for right reverse

// PWM settings
#define PWM_FREQ_HZ       1000
#define PWM_RESOLUTION    LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY      ((1 << 8) - 1)   // 255
#define PWM_SLOW_DUTY     (PWM_MAX_DUTY / 2)     // Medium speed: ~128 (50% of max)

// Target advertising data to detect
static const char target_uuid[] = "e2c56db5-dffb-48d2-b060-d0f5a71096e0"; // Unique custom UUID for your device
static const uint16_t target_manufacturer_id = 0x1234; // Your manufacturer ID
static const uint8_t target_manufacturer_data[] = {1, 2, 3, 4}; // Your custom data
static const size_t target_manufacturer_data_len = sizeof(target_manufacturer_data);

// WiFi Configuration - Connect to existing network
#define WIFI_SSID      "619"    // 🔧 CHANGE THIS to your WiFi network name
#define WIFI_PASS      "arif2022"     // 🔧 CHANGE THIS to your WiFi password  
#define WIFI_MAXIMUM_RETRY  5

// mDNS Configuration
#define MDNS_HOSTNAME  "esp32-robot"            // ESP32 will be accessible as "esp32-robot.local"

// Global variables for command handling
static char received_command[128] = {0};
static bool new_command_received = false;
static esp_ip4_addr_t esp32_ip;  // Store ESP32's IP address

// Target detection flag
static bool target_active = false;  // Flag indicating if target is currently detected
static int64_t last_target_seen = 0;  // Timestamp of last target detection
#define TARGET_TIMEOUT_MS 5000  // 5 seconds timeout for target detection

// Non-blocking motor control variables
static int64_t motor_start_time = 0;  // When current motor action started
static int64_t motor_duration = 0;    // How long the motor action should run
static bool motor_active = false;     // Whether motors are currently running a timed action

// Function declarations
void stop_motors(void);
void rotate_360_degrees(void);
void move_straight_forward(void);

// Check if target is still active (not timed out)
bool is_target_active(void) {
    // Check target timeout
    int64_t current_time = esp_timer_get_time() / 1000;  // Current time in ms
    if (target_active && (current_time - last_target_seen) > TARGET_TIMEOUT_MS) {
        target_active = false;  // Target timed out
        ESP_LOGI(TAG, "🔴 Target lost (timeout after %d seconds)", TARGET_TIMEOUT_MS / 1000);
    }
    return target_active;
}

// RSSI filter and trend detection variables
#define BUFFER_SIZE 5
bool rotated = false;  // Flag to track if vehicle has already rotated once
bool is_rotating = false;  // Flag to track if vehicle is currently rotating
int max_rssi_during_rotation = -127;  // Store max RSSI while rotating (start with min possible)
int rssiBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;
int prevAvgRSSI = 0;
const int TREND_THRESHOLD = 1; // dBm, more sensitive

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "� WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ Connected to WiFi! ESP32 IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        esp32_ip = event->ip_info.ip;
        
        ESP_LOGI(TAG, "🌐 ESP32 ready for connections!");
        ESP_LOGI(TAG, "📱 Flutter app should connect to: http://" IPSTR "/command", IP2STR(&esp32_ip));
    }
}

// Initialize WiFi in station mode
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "� Connecting to WiFi SSID: %s", WIFI_SSID);
}

// HTTP POST handler for commands from Flutter
static esp_err_t command_post_handler(httpd_req_t *req)
{
    char buf[128];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        /* Content too long */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content too long");
        return ESP_FAIL;
    }

    /* Read the data for the request */
    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request timeout");
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    ESP_LOGI(TAG, "📨 HTTP command received: %s", buf);

    // Parse JSON command
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *command = cJSON_GetObjectItem(json, "command");
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing command field");
        return ESP_FAIL;
    }

    // Copy command to global variable
    strncpy(received_command, command->valuestring, sizeof(received_command) - 1);
    received_command[sizeof(received_command) - 1] = '\0';
    new_command_received = true;

    cJSON_Delete(json);

    // Send response
    const char* response = "{\"status\":\"ok\",\"message\":\"Command received\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// HTTP GET handler for status
static esp_err_t status_get_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", "active");
    cJSON_AddBoolToObject(json, "rotating", is_rotating);
    cJSON_AddBoolToObject(json, "target_detected", is_target_active());
    cJSON_AddBoolToObject(json, "motor_active", motor_active);
    
    // Add time remaining if motors are active
    if (motor_active) {
        int64_t current_time = esp_timer_get_time() / 1000;  // Current time in ms
        int64_t elapsed_time = current_time - motor_start_time;
        int64_t remaining_time = motor_duration - elapsed_time;
        if (remaining_time < 0) remaining_time = 0;
        cJSON_AddNumberToObject(json, "time_remaining_ms", remaining_time);
    }
    
    // Convert ESP IP to string format
    char ip_str[16];
    esp_ip4addr_ntoa(&esp32_ip, ip_str, sizeof(ip_str));
    cJSON_AddStringToObject(json, "ip", ip_str);
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, HTTPD_RESP_USE_STRLEN);
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// Start HTTP server
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "🌐 Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        httpd_uri_t command_uri = {
            .uri       = "/command",
            .method    = HTTP_POST,
            .handler   = command_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &command_uri);

        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);

        ESP_LOGI(TAG, "✅ HTTP server started successfully");
        ESP_LOGI(TAG, "📡 Endpoints available:");
        ESP_LOGI(TAG, "   POST /command - Send commands via JSON");
        ESP_LOGI(TAG, "   GET  /status  - Check robot status");
        return server;
    }

    ESP_LOGE(TAG, "❌ Failed to start HTTP server");
    return NULL;
}
// Process received commands (from WiFi HTTP)
void process_flutter_command(const char* command) {
    ESP_LOGI(TAG, "🔄 Processing command: '%s'", command);
    
    // Remove any whitespace/newlines
    char clean_command[64];
    strncpy(clean_command, command, sizeof(clean_command) - 1);
    clean_command[sizeof(clean_command) - 1] = '\0';
    
    // Remove trailing whitespace
    int len = strlen(clean_command);
    while (len > 0 && (clean_command[len-1] == '\n' || clean_command[len-1] == '\r' || clean_command[len-1] == ' ')) {
        clean_command[--len] = '\0';
    }
    
    ESP_LOGI(TAG, "🔄 Clean command: '%s'", clean_command);
    
    if (strcmp(clean_command, "rotate") == 0) {
        if (!is_rotating) {
            ESP_LOGI(TAG, "📱 Command: Starting rotation...");
            rotate_360_degrees();
        } else {
            ESP_LOGI(TAG, "⚠️ Already rotating, ignoring command");
        }
    } else if (strcmp(clean_command, "stop") == 0) {
        ESP_LOGI(TAG, "📱 Command: Stopping motors...");
        stop_motors();
        is_rotating = false;
        
    } else if (strcmp(clean_command, "scan") == 0) {
        ESP_LOGI(TAG, "📱 Command: BLE scanning is already active");
    } else if (strcmp(clean_command, "forward") == 0) {
        ESP_LOGI(TAG, "📱 Command: Moving forward...");
        move_straight_forward();
    } else if (strcmp(clean_command, "deliver") == 0) {
        if (is_target_active()) {
            ESP_LOGI(TAG, "📱 Command: Starting delivery sequence!");
            if (!is_rotating) {
                ESP_LOGI(TAG, "🚚 Step 1: Rotating to scan for best direction...");
                rotate_360_degrees();
                ESP_LOGI(TAG, "🚚 Step 2: Moving forward to deliver...");
                move_straight_forward();
                ESP_LOGI(TAG, "✅ Delivery sequence completed!");
            } else {
                ESP_LOGI(TAG, "⚠️ Already rotating, ignoring deliver command");
            }
        } else {
            ESP_LOGI(TAG, "❌ Cannot deliver: No active target detected!");
            ESP_LOGI(TAG, "💡 Wait for target detection or use BLE advertising to become discoverable");
        }
    } else if (strcmp(clean_command, "meet") == 0) {
        if (is_target_active()) {
            ESP_LOGI(TAG, "📱 Command: Meeting sequence started!");
            if (!is_rotating) {
                ESP_LOGI(TAG, "🤝 Step 1: Rotating to scan for best path to you...");
                rotate_360_degrees();
                ESP_LOGI(TAG, "🤝 Step 2: Moving forward to meet you...");
                move_straight_forward();
                ESP_LOGI(TAG, "✅ Arrived at meeting point!");
            } else {
                ESP_LOGI(TAG, "⚠️ Already rotating, ignoring meet command");
            }
        } else {
            ESP_LOGI(TAG, "❌ Cannot meet: No active target detected!");
            ESP_LOGI(TAG, "💡 Wait for target detection or use BLE advertising to become discoverable");
        }
    } else if (strcmp(clean_command, "test") == 0) {
        ESP_LOGI(TAG, "📱 Command: Test successful! Motors and communication working ✅");
    } else {
        ESP_LOGI(TAG, "❓ Unknown command: '%s'", clean_command);
        ESP_LOGI(TAG, "💡 Available commands: rotate, stop, scan, forward, deliver, meet, test");
    }
}

// Motor control functions
void move_straight_forward(void) {
    ESP_LOGI(TAG, "🚗 Starting non-blocking forward movement for 10 seconds...");
    
    // Based on your rotation function, correct motor directions are:
    // LEFT motor: LPWM = forward, RPWM = reverse
    // RIGHT motor: LPWM = reverse, RPWM = forward
    
    // LEFT side forward
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_SLOW_DUTY); // LEFT_LPWM (forward)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // LEFT_RPWM (off)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // RIGHT side forward (uses RPWM for forward motion)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0); // RIGHT_LPWM (off)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, PWM_SLOW_DUTY); // RIGHT_RPWM (forward)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

    // Set up non-blocking timer
    motor_start_time = esp_timer_get_time() / 1000;  // Current time in ms
    motor_duration = 10000;  // 10 seconds
    motor_active = true;
    
    ESP_LOGI(TAG, "� Forward movement started - will auto-stop after 10 seconds");
}
void init_pwm_channels(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch = {0};
    // map channel 0 -> LEFT_LPWM
    ch.channel = LEDC_CHANNEL_0;
    ch.gpio_num = LEFT_LPWM;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.timer_sel = LEDC_TIMER_0;
    ch.duty = 0;
    ch.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // channel 1 -> LEFT_RPWM
    ch.channel = LEDC_CHANNEL_1;
    ch.gpio_num = LEFT_RPWM;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // channel 2 -> RIGHT_LPWM
    ch.channel = LEDC_CHANNEL_2;
    ch.gpio_num = RIGHT_LPWM;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // channel 3 -> RIGHT_RPWM
    ch.channel = LEDC_CHANNEL_3;
    ch.gpio_num = RIGHT_RPWM;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    // change kortesi
    // ensure all duties start at 0
    for (int c = 0; c < 4; ++c) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c);
    }
}

void stop_motors(void) {
    // Set all PWM to 0
    for (int c = 0; c < 4; ++c) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c);
    }
    
    // Clear motor timing variables
    motor_active = false;
    is_rotating = false;
    
    ESP_LOGI(TAG, "🛑 Motors stopped and timers cleared");
}

void rotate_360_degrees(void) {
    ESP_LOGI(TAG, "🔄 Starting non-blocking circular movement for 25 seconds while tracking max RSSI!");
    
    // Reset max RSSI tracker and set rotating flag
    max_rssi_during_rotation = -127;  // Reset to minimum possible RSSI
    is_rotating = true;
    
    // Move in circular path with SMALLER RADIUS: Greater speed difference = tighter turn
    // Smaller radius requires bigger speed difference between wheels
    
    // LEFT side: Very slow speed for inner wheel (smaller radius)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_SLOW_DUTY / 3); // LEFT_LPWM (33% speed)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);                // LEFT_RPWM
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // RIGHT side: Full slow speed for outer wheel  
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);                // RIGHT_LPWM
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, PWM_SLOW_DUTY);    // RIGHT_RPWM (100% slow speed)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
    ESP_LOGI(TAG, "🔄 Motors moving in circular path... tracking RSSI for 25 seconds");

    // Set up non-blocking timer
    motor_start_time = esp_timer_get_time() / 1000;  // Current time in ms
    motor_duration = 25000;  // 25 seconds
    motor_active = true;
    
    ESP_LOGI(TAG, "� Rotation started - will auto-stop after 25 seconds");
}

static int ble_app_scan_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) {
        char addr_str[BLE_HS_ADV_MAX_SZ];
        ble_addr_to_str(&event->disc.addr, addr_str);
        
        // Extract device name from advertisement data
        char device_name[32] = {0};
        const char *name = NULL;
        const struct ble_hs_adv_field *field = NULL;
        
        if (ble_hs_adv_find_field(BLE_HS_ADV_TYPE_COMP_NAME, event->disc.data, event->disc.length_data, &field) == 0) {
            int name_len = field->length > 31 ? 31 : field->length;
            memcpy(device_name, field->value, name_len);
            device_name[name_len] = '\0';
            name = device_name;
        } else if (ble_hs_adv_find_field(BLE_HS_ADV_TYPE_INCOMP_NAME, event->disc.data, event->disc.length_data, &field) == 0) {
            int name_len = field->length > 31 ? 31 : field->length;
            memcpy(device_name, field->value, name_len);
            device_name[name_len] = '\0';
            name = device_name;
        }

        // Check for target advertising data
        bool is_target_device = false;
        bool has_target_uuid = false;
        bool has_target_manufacturer = false;
        
        // Check for 128-bit UUIDs matching our custom target_uuid
        if (ble_hs_adv_find_field(BLE_HS_ADV_TYPE_COMP_UUIDS128, event->disc.data, event->disc.length_data, &field) == 0) {
            ESP_LOGI(TAG, "Found 128-bit UUID in advertisement, comparing with target: %s", target_uuid);
            ESP_LOGI(TAG, "Field length: %d bytes", field->length);
            
            // Print raw bytes for debugging
            ESP_LOGI(TAG, "Raw UUID bytes (%d): ", field->length);
            for (int i = 0; i < field->length; i++) {
                printf("%02x ", field->value[i]);
            }
            printf("\n");
            
            if (field->length >= 16) {
                ESP_LOGI(TAG, "Field length is >= 16, proceeding with comparison...");
                // Convert target_uuid string to bytes for comparison
                uint8_t target_uuid_bytes[16] = {0};
                sscanf(target_uuid,
                    "%02hhx%02hhx%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
                    &target_uuid_bytes[15], &target_uuid_bytes[14], &target_uuid_bytes[13], &target_uuid_bytes[12],
                    &target_uuid_bytes[11], &target_uuid_bytes[10], &target_uuid_bytes[9], &target_uuid_bytes[8],
                    &target_uuid_bytes[7], &target_uuid_bytes[6], &target_uuid_bytes[5], &target_uuid_bytes[4],
                    &target_uuid_bytes[3], &target_uuid_bytes[2], &target_uuid_bytes[1], &target_uuid_bytes[0]);
                if (memcmp(field->value, target_uuid_bytes, 16) == 0) {
                    has_target_uuid = true;
                    ESP_LOGI(TAG, "Found target 128-bit UUID matching %s", target_uuid);
                } else {
                    // Print the first 16 bytes as UUID of the non-target device
                    ESP_LOGI(TAG, "Non-target 128-bit UUID found: %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                        field->value[15], field->value[14], field->value[13], field->value[12],
                        field->value[11], field->value[10], field->value[9], field->value[8],
                        field->value[7], field->value[6], field->value[5], field->value[4],
                        field->value[3], field->value[2], field->value[1], field->value[0]);
                }
            } else {
                ESP_LOGI(TAG, "Field length is less than 16 (actual: %d), skipping comparison", field->length);
            }
        }
        
        // Check for manufacturer data
        if (ble_hs_adv_find_field(BLE_HS_ADV_TYPE_MFG_DATA, event->disc.data, event->disc.length_data, &field) == 0) {
            if (field->length >= 2) {
                // First 2 bytes are manufacturer ID (little endian)
                uint16_t mfg_id = field->value[0] | (field->value[1] << 8);
                
               // ESP_LOGI(TAG, "Device %s has manufacturer ID: 0x%04X", addr_str, mfg_id);
                
                if (mfg_id == target_manufacturer_id) {
                    // Check if manufacturer data matches
                    if (field->length >= (2 + target_manufacturer_data_len)) {
                        if (memcmp(&field->value[2], target_manufacturer_data, target_manufacturer_data_len) == 0) {
                            has_target_manufacturer = true;
                            ESP_LOGI(TAG, ">>> FOUND TARGET MANUFACTURER DATA!");
                        }
                    }
                }
            }
        }
        
        // Device is target if it matches MAC address OR has target advertising data
        is_target_device = (has_target_uuid || has_target_manufacturer);

        if (is_target_device) {
            int rssi = event->disc.rssi;
            
            // Update last seen timestamp
            last_target_seen = esp_timer_get_time() / 1000;  // Convert to milliseconds
            
            // If we're rotating, track the max RSSI
            if (is_rotating) {
                if (rssi > max_rssi_during_rotation) {
                    max_rssi_during_rotation = rssi;
                    ESP_LOGI(TAG, "🔄📶 NEW MAX RSSI while moving in circle: %d dBm", max_rssi_during_rotation);
                }
            }
            
            // Always log target detection immediately for debugging
            ESP_LOGI(TAG, "🎯 TARGET DETECTED: %s | Name: %s | RSSI: %d dBm | UUID: %s | MFG: %s", 
                     addr_str, name ? name : "(none)", rssi,
                     has_target_uuid ? "YES" : "NO",
                     has_target_manufacturer ? "YES" : "NO");
            
            // Continue with RSSI buffering for smooth movement
            rssiBuffer[bufferIndex] = rssi;
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
            if (bufferIndex == 0) bufferFilled = true;
            
            // Show buffer progress for debugging
            ESP_LOGI(TAG, "📊 RSSI Buffer: %d/%d readings collected", 
                     bufferFilled ? BUFFER_SIZE : bufferIndex, BUFFER_SIZE);

            if (bufferFilled) {
                long sum = 0;
                for (int j = 0; j < BUFFER_SIZE; j++) sum += rssiBuffer[j];
                int avgRSSI = sum / BUFFER_SIZE;

                ESP_LOGI(TAG, ">>> SMOOTH TARGET FOUND: %s | Name: %s | Avg RSSI: %d dBm | UUID: %s | MFG: %s | Rotating: %s", 
                         addr_str, name ? name : "(none)", avgRSSI,
                         has_target_uuid ? "YES" : "NO",
                         has_target_manufacturer ? "YES" : "NO",
                         is_rotating ? "YES" : "NO");

                // Set target active flag when stable target is detected
                if (!rotated && !is_rotating) {  // First stable detection
                    rotated = true;  // Mark that we have detected once
                    target_active = true;  // Set target as active
                    ESP_LOGI(TAG, "🎯 TARGET ACTIVATED! Ready for commands.");
                    ESP_LOGI(TAG, "📱 You can now send 'meet' or 'deliver' commands from Flutter app.");
                } else if (!is_rotating) {
                    target_active = true;  // Keep target active while detected
                    ESP_LOGI(TAG, "🔄 Target still active and ready for commands");
                }

                int diff = avgRSSI - prevAvgRSSI;
                if (diff > TREND_THRESHOLD) {
                    ESP_LOGI(TAG, "Trend: Getting Closer ✓");
                } else if (diff < -TREND_THRESHOLD) {
                    ESP_LOGI(TAG, "Trend: Moving Away ✗");
                } else {
                    ESP_LOGI(TAG, "Trend: Stable ➡");
                }
                prevAvgRSSI = avgRSSI;
            }
        } else {
            //ESP_LOGI(TAG, "Discovered device: %s, Name: %s, RSSI: %d", addr_str, name ? name : "(none)", event->disc.rssi);
        }
    }
    return 0;
}

/* Start scanning */
static void ble_app_scan(void) {
    struct ble_gap_disc_params disc_params = {0};
    
    // Optimize scanning for faster target detection
    disc_params.filter_duplicates = 0;    // Allow duplicate advertisements for faster buffer filling
    disc_params.passive = 1;              // Passive scanning (less power, faster)
    disc_params.itvl = 0x10;              // Scan interval: 16 * 0.625ms = 10ms (fast scanning)
    disc_params.window = 0x10;            // Scan window: 16 * 0.625ms = 10ms (100% duty cycle)
    
    ESP_LOGI(TAG, "Starting optimized BLE scanning (no duplicate filter, fast intervals)...");

    int rc = ble_gap_disc(0, BLE_HS_FOREVER, &disc_params, ble_app_scan_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Started BLE scanning with optimized parameters");
    }
}

/* Called when the BLE stack is reset */
static void on_stack_reset(int reason) {
    ESP_LOGI(TAG, "BLE stack reset, reason=%d", reason);
}

/* Called when the BLE stack syncs */
static void on_stack_sync(void) {
    ESP_LOGI(TAG, "BLE stack synced");
    ble_app_scan();
}

/* Configure NimBLE host */
static void nimble_host_config_init(void) {
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    // ble_store_config_init(); // Uncomment if available in your NimBLE version
}

/* NimBLE host task */
static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task started");
    nimble_port_run();
    vTaskDelete(NULL);
}

/* Check if motor timer has expired and auto-stop if needed */
static void check_motor_timer(void) {
    if (motor_active) {
        int64_t current_time = esp_timer_get_time() / 1000;  // Current time in ms
        int64_t elapsed_time = current_time - motor_start_time;
        
        if (elapsed_time >= motor_duration) {
            // Timer expired, stop motors
            stop_motors();
            
            if (is_rotating) {
                ESP_LOGI(TAG, "🛑 25-second circular movement completed! Motors auto-stopped.");
                ESP_LOGI(TAG, "📶 MAX RSSI during circular movement: %d dBm", max_rssi_during_rotation);
            } else {
                ESP_LOGI(TAG, "🛑 Forward movement completed! Motors auto-stopped.");
            }
        }
    }
}

/* Command monitoring task */
static void command_monitor_task(void *param) {
    ESP_LOGI(TAG, "🔍 Command monitor task started - checking every 100ms");
    
    while (1) {
        // Check if motors need to be auto-stopped
        check_motor_timer();
        
        // Process new commands
        if (new_command_received) {
            ESP_LOGI(TAG, "🚨 NEW COMMAND DETECTED! Processing: '%s'", received_command);
            process_flutter_command(received_command);
            new_command_received = false;
            ESP_LOGI(TAG, "✅ Command processing completed, flag cleared");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize motor control PWM channels
    ESP_LOGI(TAG, "🔧 Initializing motor control...");
    init_pwm_channels();
    ESP_LOGI(TAG, "✅ Motor control initialized");

    // Initialize WiFi as station (connect to existing network)
    ESP_LOGI(TAG, "� Connecting to WiFi network...");
    wifi_init_sta();
    
    // Wait for WiFi connection (give it some time)
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Start HTTP server
    ESP_LOGI(TAG, "🚀 Starting HTTP server...");
    start_webserver();
    
    // Initialize BLE scanning for target detection
    ESP_LOGI(TAG, "🔍 Initializing BLE scanning for target detection...");
    ret = nimble_port_init();
    ESP_ERROR_CHECK(ret);
    nimble_host_config_init();
    
    // Create tasks
    xTaskCreate(nimble_host_task, "nimble_host", 4096, NULL, 5, NULL);
    xTaskCreate(command_monitor_task, "cmd_monitor", 2048, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "🚗 Robot initialized successfully!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📱 FLUTTER CONNECTION:");
    ESP_LOGI(TAG, "   1. Make sure your phone is on the same WiFi network: %s", WIFI_SSID);
    ESP_LOGI(TAG, "   2. ESP32 IP will be shown above when connected");
    ESP_LOGI(TAG, "   3. Flutter app should send POST to: http://ESP32_IP/command");
    ESP_LOGI(TAG, "   4. Test with GET: http://ESP32_IP/status");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🎯 BLE TARGET DETECTION:");
    ESP_LOGI(TAG, "   - Scanning for UUID: %s", target_uuid);
    ESP_LOGI(TAG, "   - Will auto-rotate and move when target detected");
    ESP_LOGI(TAG, "   - Manual commands can override automatic behavior");
    ESP_LOGI(TAG, "");
}


// & 'F:\esp-idf\export.ps1'; 
// idf.py build