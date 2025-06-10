#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs_adv.h"

#define TAG "HELMET_SENSOR"

// BLE Configuration
#define GATT_SERVICE_UUID        0x1234  // Custom service UUID
#define GATT_CHAR_VOLTAGE_UUID   0x1235  // Custom voltage characteristic UUID
#define DEVICE_NAME             "HELMET-SENSOR"
#define VOLTAGE_THRESHOLD       0.50f    // Lowered threshold to 0.80V for testing
#define FORCE_ADVERTISING       true     // Force advertising for testing

// ADC Configuration
#define ADC_CHANNEL            ADC_CHANNEL_0  // ADC1 Channel 0
#define ADC_ATTEN             ADC_ATTEN_DB_12  // Updated from DB_11 to DB_12
#define ADC_WIDTH             ADC_BITWIDTH_12
#define ADC_SAMPLE_PERIOD_MS  100  // Read ADC every 100ms

// State variables
static struct {
    uint16_t conn_handle;
    bool ble_connected;
    uint16_t voltage_val_handle;
    float current_voltage;
    bool is_advertising;
    TimerHandle_t notification_timer;
} state = {
    .conn_handle = BLE_HS_CONN_HANDLE_NONE,
    .ble_connected = false,
    .voltage_val_handle = 0,
    .current_voltage = 0.0f,
    .is_advertising = false,
    .notification_timer = NULL
};

// ADC handles
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle;

// BLE UUIDs
static const ble_uuid16_t generic_access_svc_uuid = BLE_UUID16_INIT(0x1800);
static const ble_uuid16_t device_name_char_uuid = BLE_UUID16_INIT(0x2A00);
static const ble_uuid16_t custom_svc_uuid = BLE_UUID16_INIT(GATT_SERVICE_UUID);
static const ble_uuid16_t voltage_char_uuid = BLE_UUID16_INIT(GATT_CHAR_VOLTAGE_UUID);

// Function declarations
static void init_adc(void);
static void read_adc_task(void *pvParameters);
static void send_voltage_notification(TimerHandle_t timer);
static void start_advertising(void);
static void stop_advertising(void);
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_host_task(void *param);
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static int ble_gap_event(struct ble_gap_event *event, void *arg);

// GATT Characteristics
static const struct ble_gatt_chr_def generic_access_chrs[] = {
    {
        .uuid = (ble_uuid_t *)&device_name_char_uuid,
        .access_cb = gatt_svr_chr_access,
        .flags = BLE_GATT_CHR_F_READ,
    },
    { 0 }
};

static const struct ble_gatt_chr_def custom_svc_chrs[] = {
    {
        .uuid = (ble_uuid_t *)&voltage_char_uuid,
        .access_cb = gatt_svr_chr_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &state.voltage_val_handle,
    },
    { 0 }
};

// Initialize ADC
static void init_adc(void) {
    // ADC1 init
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // ADC1 config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));

    // ADC calibration
    adc_cali_handle_t handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &handle));
    adc_cali_handle = handle;

    ESP_LOGI(TAG, "ADC initialized successfully");
}

// Read ADC continuously
static void read_adc_task(void *pvParameters) {
    int adc_raw;
    int voltage_mv;
    float voltage_v;

    while (1) {
        // Read ADC
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv));
        
        // Convert to volts
        voltage_v = voltage_mv / 1000.0f;
        state.current_voltage = voltage_v;

        // Log voltage reading with more detail
        ESP_LOGI(TAG, "Voltage: %.2fV (Raw: %d, mV: %d) - %s", 
                 voltage_v, adc_raw, voltage_mv,
                 state.ble_connected ? "CONNECTED" : 
                 state.is_advertising ? "ADVERTISING" : "DISCONNECTED");

        // Check threshold and update advertising state
        if ((voltage_v >= VOLTAGE_THRESHOLD || FORCE_ADVERTISING) && !state.is_advertising) {
            ESP_LOGI(TAG, "Starting BLE advertising - Voltage: %.2fV (Threshold: %.2fV, Force: %s)", 
                     voltage_v, VOLTAGE_THRESHOLD, FORCE_ADVERTISING ? "Yes" : "No");
            start_advertising();
        } else if (voltage_v < VOLTAGE_THRESHOLD && !FORCE_ADVERTISING && state.is_advertising) {
            ESP_LOGI(TAG, "Stopping BLE advertising - Voltage: %.2fV (Threshold: %.2fV)", 
                     voltage_v, VOLTAGE_THRESHOLD);
            stop_advertising();
        }

        vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_PERIOD_MS));
    }
}

// Send voltage notification
static void send_voltage_notification(TimerHandle_t timer) {
    if (!state.ble_connected || state.voltage_val_handle == 0) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&state.current_voltage, 
                                              sizeof(state.current_voltage));
    if (!om) {
        ESP_LOGE(TAG, "Failed to create mbuf for notification");
        return;
    }

    int rc = ble_gatts_notify_custom(state.conn_handle, state.voltage_val_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error sending notification: %d", rc);
    } else {
        ESP_LOGI(TAG, "Sent voltage: %.2fV", state.current_voltage);
    }
}

// Start BLE advertising
static void start_advertising(void) {
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .tx_pwr_lvl_is_present = 1,
        .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
        .name = (const uint8_t *)DEVICE_NAME,
        .name_len = strlen(DEVICE_NAME),
        .name_is_complete = 1,
    };

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields; rc=%d", rc);
        return;
    }

    uint8_t addr_type;
    rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error inferring BLE address type: %d", rc);
        return;
    }

    // Print the device's BLE address
    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    }

    rc = ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising; rc=%d", rc);
        return;
    }

    state.is_advertising = true;
    ESP_LOGI(TAG, "BLE advertising started with name: %s", DEVICE_NAME);
}

// Stop BLE advertising
static void stop_advertising(void) {
    int rc = ble_gap_adv_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Error stopping advertising; rc=%d", rc);
        return;
    }

    state.is_advertising = false;
    ESP_LOGI(TAG, "BLE advertising stopped");
}

// GATT characteristic access
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    uint16_t uuid16 = ble_uuid_u16(uuid);
    int rc;

    switch (uuid16) {
        case 0x2A00: // Device Name
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, DEVICE_NAME, strlen(DEVICE_NAME));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;

        case GATT_CHAR_VOLTAGE_UUID:
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, &state.current_voltage, 
                                  sizeof(state.current_voltage));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// BLE GAP event handler
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                state.conn_handle = event->connect.conn_handle;
                state.ble_connected = true;
                ESP_LOGI(TAG, "BLE connected. Connection handle: %d", state.conn_handle);
                
                // Start notification timer
                if (state.notification_timer == NULL) {
                    state.notification_timer = xTimerCreate(
                        "notify_timer",
                        pdMS_TO_TICKS(3000),  // 3 seconds
                        pdTRUE,               // Auto-reload
                        0,                    // Timer ID
                        send_voltage_notification
                    );
                    xTimerStart(state.notification_timer, 0);
                }
            } else {
                ESP_LOGE(TAG, "Error: Connection failed; status=%d", 
                         event->connect.status);
                state.ble_connected = false;
                state.conn_handle = BLE_HS_CONN_HANDLE_NONE;
                if (state.current_voltage >= VOLTAGE_THRESHOLD) {
                    start_advertising();
                }
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Disconnected. Reason: %d", event->disconnect.reason);
            state.ble_connected = false;
            state.conn_handle = BLE_HS_CONN_HANDLE_NONE;
            
            // Stop notification timer
            if (state.notification_timer != NULL) {
                xTimerStop(state.notification_timer, 0);
            }
            
            if (state.current_voltage >= VOLTAGE_THRESHOLD) {
                start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event; conn_handle=%d, attr_handle=%d, reason=%d",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason);
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
            if (state.current_voltage >= VOLTAGE_THRESHOLD) {
                start_advertising();
            }
            return 0;
    }
    return 0;
}

// BLE host task
static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// BLE sync callback
static void ble_app_on_sync(void) {
    int rc;

    // Set the default device name
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    // Initialize services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register GATT services
    static const struct ble_gatt_svc_def gatt_svcs[] = {
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = (ble_uuid_t *)&generic_access_svc_uuid,
            .characteristics = generic_access_chrs,
        },
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = (ble_uuid_t *)&custom_svc_uuid,
            .characteristics = custom_svc_chrs,
        },
        { 0 }
    };

    rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    // Start advertising if voltage is above threshold or forced
    if (state.current_voltage >= VOLTAGE_THRESHOLD || FORCE_ADVERTISING) {
        ESP_LOGI(TAG, "Starting initial advertising - Voltage: %.2fV (Threshold: %.2fV, Force: %s)",
                 state.current_voltage, VOLTAGE_THRESHOLD, FORCE_ADVERTISING ? "Yes" : "No");
        start_advertising();
    }
}

// BLE reset callback
static void ble_app_on_reset(int reason) {
    ESP_LOGI(TAG, "Resetting state; reason=%d", reason);
    ble_app_on_sync();
}

// Main application entry point
void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize ADC
    init_adc();

    // Initialize NimBLE
    ESP_ERROR_CHECK(nimble_port_init());

    // Initialize the NimBLE host configuration
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    // Set the default device name
    ble_svc_gap_device_name_set(DEVICE_NAME);

    // Initialize services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Set the default security parameters
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_bonding = 0;

    // Start the NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    // Create ADC reading task
    xTaskCreate(read_adc_task, "adc_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Helmet Sensor Started (ESP32-C3 Seeed Studio)");
    ESP_LOGI(TAG, "Voltage threshold: %.2fV", VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "BLE device name: %s", DEVICE_NAME);
}
