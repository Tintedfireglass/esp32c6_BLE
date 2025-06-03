#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "nvs_flash.h"

/* NimBLE headers */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

// BLE settings
static const char *device_name = "VEHICLE-START";
static bool ble_active = false;
static adc_oneshot_unit_handle_t adc1_handle;

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

// Initialize ADC
void init_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
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
    // Initialize ADC
    init_adc();
    
    // Initialize BLE
    init_ble();
    
    printf("MQ3 BLE Monitor Started (ESP32-C6)\n");
    
    while (1) {
        // Read MQ3 sensor (ADC value 0-4095)
        int adc_value;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_value));
        
        float percentage = (adc_value / 4095.0) * 100;
        
        printf("MQ3 Reading: %d (%.1f%%)\n", adc_value, percentage);
        
        if (percentage > 50.0) {
            printf("Threshold exceeded! Stopping BLE advertising\n");
            stop_ble_advertising();
        } else {
            printf("Safe level! Starting BLE advertising - VEHICLE START\n");
            start_ble_advertising_vehicle();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
    }
}