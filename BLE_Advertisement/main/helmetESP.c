#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"

// ADC Configuration
#define ADC1_CHANNEL ADC1_CHANNEL_4  // GPIO2 on XIAO ESP32-C3
#define THRESHOLD_VOLTAGE 1.3f  // Midpoint between 0.7V and 1.9V
static esp_adc_cal_characteristics_t *adc_chars;

/* NimBLE headers */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

// BLE settings
static const char *device_name = "VEHICLE-START";
static bool ble_active = false;

static bool digital_input_state = false;

// Voltage reading and state
static float voltage = 0.0f;
static bool should_advertise = false;
static uint32_t last_state_change_time = 0;
#define DEBOUNCE_DELAY_MS 3000  // 3 second debounce delay
#define VOLTAGE_THRESHOLD 1.3f  // Voltage threshold in volts

// BLE event handler
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_ADV_COMPLETE:
            printf("Advertising stopped\n");
            ble_active = false;
            break;
        default:
            break;
    }
    return 0;
}

// Initialize ADC for digital threshold
void init_adc() {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_11);  // 0-3.1V range
    
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("eFuse Two Point: Supported\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("Using Default Vref: 1100mV\n");
    }
}

// Start BLE advertising
void start_ble_advertising_vehicle() {
    if (ble_active) {
        return;
    }

    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    // Clear the advertising data structure
    memset(&fields, 0, sizeof(fields));
    
    // Set the advertising data
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        printf("Error setting advertisement fields; rc=%d\n", rc);
        return;
    }

    // Configure and start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        printf("Error enabling advertisement; rc=%d\n", rc);
        return;
    }
    
    ble_active = true;
    printf("BLE advertising started\n");
}

// Stop BLE advertising
void stop_ble_advertising() {
    if (!ble_active) {
        return;
    }
    
    int rc = ble_gap_adv_stop();
    if (rc != 0) {
        printf("Error stopping advertisement; rc=%d\n", rc);
        return;
    }
    
    ble_active = false;
    printf("BLE advertising stopped\n");
}

// Called when BLE stack is synchronized
static void ble_app_on_sync(void) {
    printf("Bluetooth initialized\n");
    start_ble_advertising_vehicle();
}

// Task that runs the BLE host
static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Initialize BLE
void init_ble(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the NimBLE host configuration
    nimble_port_init();
    
    // Initialize the NimBLE host
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    
    // Set the default device name
    int rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);
    
    // Start the task handling the NimBLE host
    nimble_port_freertos_init(ble_host_task);
}

void app_main() {
    
    init_adc();
    init_ble();
    
    printf("Helmet Detection Started (ESP32-C3 Seeed Studio)\n");
    printf("Advertising when voltage >= %.2fV\n", VOLTAGE_THRESHOLD);
    
    // Initial read
    int raw = adc1_get_raw(ADC1_CHANNEL);
    voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars) / 1000.0f;
    should_advertise = (voltage >= VOLTAGE_THRESHOLD);
    printf("Initial voltage: %.2fV - %s\n", voltage, 
           should_advertise ? "ADVERTISING" : "NOT advertising");
    
    if (should_advertise) {
        start_ble_advertising_vehicle();
    }
    
    uint32_t last_print = 0;
    
    while (1) {
        // Read raw ADC value and convert to voltage
        raw = adc1_get_raw(ADC1_CHANNEL);
        voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars) / 1000.0f;
        
        // Get current time
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check if we should change state
        bool new_should_advertise = (voltage >= VOLTAGE_THRESHOLD);
        
        // Only process state change if it's different from current state
        if (new_should_advertise != should_advertise) {
            // Only change state if the reading is stable for DEBOUNCE_DELAY_MS
            if ((current_time - last_state_change_time) >= DEBOUNCE_DELAY_MS) {
                should_advertise = new_should_advertise;
                last_state_change_time = current_time;
                
                if (should_advertise) {
                    printf("Voltage %.2fV >= %.2fV - Starting BLE advertising\n", 
                           voltage, VOLTAGE_THRESHOLD);
                    start_ble_advertising_vehicle();
                } else {
                    printf("Voltage %.2fV < %.2fV - Stopping BLE advertising\n",
                           voltage, VOLTAGE_THRESHOLD);
                    stop_ble_advertising();
                }
            }
        } else {
            // Reset the timer if we're not trying to change state
            last_state_change_time = current_time;
        }
        
        // Debug output (every 2 seconds)
        if ((current_time - last_print) >= 2000) {
            printf("Voltage: %.2fV - %s\n", voltage, 
                   should_advertise ? "ADVERTISING" : "NOT advertising");
            last_print = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
    }
}
