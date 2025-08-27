#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

// WiFi and HTTP server includes
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "cJSON.h"
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

// WiFi Configuration
#define WIFI_SSID      "robot"     // Change to your WiFi network
#define WIFI_PASS      "robot123"     // Change to your WiFi password
#define MAX_STA_CONN   4

// Global variables for command handling
static char received_command[128] = {0};
static bool new_command_received = false;

// Function declarations
void stop_motors(void);
void rotate_360_degrees(void);


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
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station joined, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station left, AID=%d", event->aid);
    }
}

// Initialize WiFi in Access Point mode
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
    
    // Get and display IP address
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif) {
        esp_netif_get_ip_info(netif, &ip_info);
        ESP_LOGI(TAG, "ESP32 AP IP Address: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "ESP32 AP Gateway: " IPSTR, IP2STR(&ip_info.gw));
        ESP_LOGI(TAG, "ESP32 AP Netmask: " IPSTR, IP2STR(&ip_info.netmask));
    }
}

// HTTP POST handler for receiving commands from Flutter
static esp_err_t command_post_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request timeout");
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    
    ESP_LOGI(TAG, "📱 Raw received data: %s", buf);
    
    // Parse JSON to extract command
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        ESP_LOGE(TAG, "❌ Failed to parse JSON");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *command_json = cJSON_GetObjectItem(json, "command");
    if (command_json == NULL || !cJSON_IsString(command_json)) {
        ESP_LOGE(TAG, "❌ No 'command' field found in JSON");
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing command field");
        return ESP_FAIL;
    }
    
    // Extract just the command value
    const char *command_value = cJSON_GetStringValue(command_json);
    strncpy(received_command, command_value, sizeof(received_command) - 1);
    received_command[sizeof(received_command) - 1] = '\0';
    new_command_received = true;
    
    ESP_LOGI(TAG, "📱 Parsed command: %s", received_command);
    
    cJSON_Delete(json);

    // Send response back to Flutter
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Command received", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// Start HTTP server
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t command_uri = {
            .uri       = "/command",
            .method    = HTTP_POST,
            .handler   = command_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &command_uri);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

// Process received commands
void process_flutter_command(const char* command) {
    ESP_LOGI(TAG, "🔄 Processing command: %s", command);
    
    if (strcmp(command, "rotate") == 0) {
        if (!is_rotating) {
            ESP_LOGI(TAG, "📱 Flutter command: Starting rotation...");
            rotate_360_degrees();
        } else {
            ESP_LOGI(TAG, "⚠️ Already rotating, ignoring command");
        }
    } else if (strcmp(command, "stop") == 0) {
        ESP_LOGI(TAG, "📱 Flutter command: Stopping motors...");
        stop_motors();
        is_rotating = false;
    } else if (strcmp(command, "scan") == 0) {
        ESP_LOGI(TAG, "📱 Flutter command: BLE scanning is already active");
    } else {
        ESP_LOGI(TAG, "❓ Unknown command: %s", command);
    }
}

// Motor control functions
void move_straight_forward(void) {
    ESP_LOGI(TAG, "🚗 Moving straight forward for 5 seconds...");
    
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

    vTaskDelay(pdMS_TO_TICKS(10000)); // Move straight for 10 seconds

    stop_motors();
    ESP_LOGI(TAG, "🛑 Stopped after moving straight.");
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
}

void rotate_360_degrees(void) {
    ESP_LOGI(TAG, "🔄 Starting SLOW circular movement for 15 seconds while tracking max RSSI!");
    
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
    
    ESP_LOGI(TAG, "🔄 Motors moving in circular path... tracking RSSI for 15 seconds");

    // Move in circle for 25 seconds to complete full 360-degree turn (BLE scanning continues in background)
    vTaskDelay(pdMS_TO_TICKS(25000));

    // Stop motors immediately after 25 seconds
    stop_motors();
    is_rotating = false;  // Clear rotating flag

    ESP_LOGI(TAG, "🛑 25-second circular movement completed! Motors stopped.");
    ESP_LOGI(TAG, "📶 MAX RSSI during circular movement: %d dBm", max_rssi_during_rotation);
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
            if (field->length == 16) {
                ESP_LOGI(TAG, "Field length is 16, proceeding with comparison...");
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
                    // Print the UUID of the non-target device
                    ESP_LOGI(TAG, "Non-target 128-bit UUID found: %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                        field->value[15], field->value[14], field->value[13], field->value[12],
                        field->value[11], field->value[10], field->value[9], field->value[8],
                        field->value[7], field->value[6], field->value[5], field->value[4],
                        field->value[3], field->value[2], field->value[1], field->value[0]);
                }
            } else {
                ESP_LOGI(TAG, "Field length is not 16 (actual: %d), skipping comparison", field->length);
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
            
            // If we're rotating, track the max RSSI
            if (is_rotating) {
                if (rssi > max_rssi_during_rotation) {
                    max_rssi_during_rotation = rssi;
                    ESP_LOGI(TAG, "🔄📶 NEW MAX RSSI while moving in circle: %d dBm", max_rssi_during_rotation);
                }
            }
            
            rssiBuffer[bufferIndex] = rssi;
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
            if (bufferIndex == 0) bufferFilled = true;

            if (bufferFilled) {
                long sum = 0;
                for (int j = 0; j < BUFFER_SIZE; j++) sum += rssiBuffer[j];
                int avgRSSI = sum / BUFFER_SIZE;

                ESP_LOGI(TAG, ">>> TARGET FOUND: %s | Name: %s | Avg RSSI: %d dBm | UUID: %s | MFG: %s | Rotating: %s", 
                         addr_str, name ? name : "(none)", avgRSSI,
                         has_target_uuid ? "YES" : "NO",
                         has_target_manufacturer ? "YES" : "NO",
                         is_rotating ? "YES" : "NO");

                // Trigger SLOW 360-degree rotation ONLY on first detection
                if (!rotated && !is_rotating) {  // Don't start new rotation if already rotated or currently rotating
                    rotated = true;  // Mark that we have rotated once
                    ESP_LOGI(TAG, "🎯 FIRST TARGET DETECTION! Starting slow 15-second circular movement...");
                    rotate_360_degrees();
                    ESP_LOGI(TAG, "✅ Circular movement complete. Now moving straight for 5 seconds...");
                    move_straight_forward();
                    ESP_LOGI(TAG, "✅ Straight movement complete. Will not move again until restart.");
                } else if (!is_rotating) {
                    ESP_LOGI(TAG, "🔄 Target still detected (already rotated once)");
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
    disc_params.filter_duplicates = 1;

    int rc = ble_gap_disc(0, BLE_HS_FOREVER, &disc_params, ble_app_scan_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Started BLE scanning");
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

/* Command monitoring task */
static void command_monitor_task(void *param) {
    while (1) {
        if (new_command_received) {
            process_flutter_command(received_command);
            new_command_received = false;
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
    ESP_LOGI(TAG, "Initializing motor control...");
    init_pwm_channels();
    ESP_LOGI(TAG, "Motor control initialized ✓");

    // Initialize WiFi Access Point
    ESP_LOGI(TAG, "Initializing WiFi AP...");
    wifi_init_softap();
    
    // Start HTTP server for Flutter commands
    ESP_LOGI(TAG, "Starting HTTP server...");
    start_webserver();
    ESP_LOGI(TAG, "HTTP server started on port 80");
    ESP_LOGI(TAG, "📱 Flutter can send commands to: http://192.168.4.1/command");

    // Initialize BLE
    ret = nimble_port_init();
    ESP_ERROR_CHECK(ret);

    nimble_host_config_init();

    // Create tasks
    xTaskCreate(nimble_host_task, "nimble_host", 4096, NULL, 5, NULL);
    xTaskCreate(command_monitor_task, "cmd_monitor", 2048, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "🚗 Robot initialized! Ready for BLE scanning and Flutter commands");
}


// & 'F:\esp-idf\export.ps1'; 
// idf.py build