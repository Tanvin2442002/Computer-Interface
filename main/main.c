#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
// static const char target_addr[] = "59:7F:F7:BE:E8:B6";
//static const char target_addr[] = "64:5D:F4:FA:6E:B9";
static const char target_addr[] = "64:5F:BB:0B:F4:42";


// RSSI filter and trend detection variables
#define BUFFER_SIZE 5
int rssiBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;
int prevAvgRSSI = 0;
const int TREND_THRESHOLD = 1; // dBm, more sensitive

static int ble_app_scan_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) {
        char addr_str[BLE_HS_ADV_MAX_SZ];
        ble_addr_to_str(&event->disc.addr, addr_str);
        // Extract device name from advertisement data
        char device_name[32] = {0}; // Buffer for device name
        const char *name = NULL;
        const struct ble_hs_adv_field *field = NULL;
        
        if (ble_hs_adv_find_field(BLE_HS_ADV_TYPE_COMP_NAME, event->disc.data, event->disc.length_data, &field) == 0) {
            // Copy name with proper length handling
            int name_len = field->length > 31 ? 31 : field->length;
            memcpy(device_name, field->value, name_len);
            device_name[name_len] = '\0';
            name = device_name;
        } else if (ble_hs_adv_find_field(BLE_HS_ADV_TYPE_INCOMP_NAME, event->disc.data, event->disc.length_data, &field) == 0) {
            // Copy name with proper length handling
            int name_len = field->length > 31 ? 31 : field->length;
            memcpy(device_name, field->value, name_len);
            device_name[name_len] = '\0';
            name = device_name;
        }

        if (strcmp(addr_str, target_addr) == 0) {
            int rssi = event->disc.rssi;
            rssiBuffer[bufferIndex] = rssi;
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
            if (bufferIndex == 0) bufferFilled = true;

            if (bufferFilled) {
                long sum = 0;
                for (int j = 0; j < BUFFER_SIZE; j++) sum += rssiBuffer[j];
                int avgRSSI = sum / BUFFER_SIZE;

                ESP_LOGI(TAG, ">>> Target %s | Name: %s | Avg RSSI: %d dBm | Prev Avg: %d dBm", addr_str, name ? name : "(none)", avgRSSI, prevAvgRSSI);

                int diff = avgRSSI - prevAvgRSSI;
                if (diff > TREND_THRESHOLD) {
                    ESP_LOGI(TAG, "Trend: Getting Closer \xE2\x9C\x94");
                } else if (diff < -TREND_THRESHOLD) {
                    ESP_LOGI(TAG, "Trend: Moving Away \xE2\x9C\x98");
                } else {
                    ESP_LOGI(TAG, "Trend: Stable \xE2\x9C\x85");
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

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    ESP_ERROR_CHECK(ret);

    nimble_host_config_init();

    xTaskCreate(nimble_host_task, "nimble_host", 4096, NULL, 5, NULL);
}
