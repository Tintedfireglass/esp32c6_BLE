#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

// Scan parameters
static uint8_t own_addr_type;

// Convert BLE address to string
static char* addr_str(const void *addr)
{
    static char buf[6 * 2 + 5 + 1];
    const uint8_t *u8p = addr;
    
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
            u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
    
    return buf;
}

// Print advertising data (simplified to show only name and MAC)
static void print_adv_data(const struct ble_hs_adv_fields *fields, const uint8_t *addr)
{
    // Print MAC address
    printf("MAC: %s ", addr_str(addr));
    
    // Print device name if available
    if (fields->name != NULL) {
        printf(" | Name: %s", fields->name);
    } else {
        printf(" | Name: (unknown)");
    }
    
    printf("\n");
}

// Called when an advertisement is received
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    static uint32_t last_print = 0;
    struct ble_hs_adv_fields fields;
    int rc;
    
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        // Parse the advertising data
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data);
        if (rc != 0) {
            return 0;
        }
        
        // Print simplified device info
        print_adv_data(&fields, event->disc.addr.val);
        

        break;
        
    case BLE_GAP_EVENT_DISC_COMPLETE:
        printf("\nScan complete. Waiting before next scan...\n");
        
        // Add a delay before restarting scan
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Restart scanning with same parameters
        struct ble_gap_disc_params disc_params = {
            .itvl = 0x60,
            .window = 0x30,
            .filter_policy = 0,
            .limited = 0,
            .passive = 0,
            .filter_duplicates = 1,
        };
        ble_gap_disc(own_addr_type, 3000, &disc_params, gap_event_cb, NULL);
        break;
        
    default:
        break;
    }
    
    return 0;
}

// Start BLE scanning
static void start_scan(void)
{
    // Set scan parameters
    struct ble_gap_disc_params disc_params = {
        .itvl = 0x60,    // Scan interval: 60ms (slower to reduce CPU load)
        .window = 0x30,  // Scan window: 30ms (50% duty cycle)
        .filter_policy = 0,  // Accept all advertisements
        .limited = 0,        // Not limited discovery
        .passive = 0,        // Active scanning (to get scan response data)
        .filter_duplicates = 1,  // Filter duplicates to reduce output
    };
    
    // Start scanning
    int rc = ble_gap_disc(own_addr_type, 3000, &disc_params,  // Scan for 3 seconds
                         gap_event_cb, NULL);
    if (rc != 0) {
        printf("Error starting scan: %d\n", rc);
        return;
    }
    
    printf("Scanning for BLE devices...\n");
}

// Called when BLE host task starts
static void ble_host_task(void *param)
{
    printf("BLE: Starting NimBLE host task\n");
    nimble_port_run();
    printf("BLE: nimble_port_run() completed\n");
    nimble_port_freertos_deinit();
    printf("BLE: Host task finished\n");
}

// Application callback for BLE host sync
static int ble_app_on_sync(void)
{
    printf("BLE: Host sync started\n");
    
    // Figure out address to use
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        printf("BLE: Failed to ensure address: %d\n", rc);
        return rc;
    }
    
    // Get our address
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        printf("BLE: Failed to copy address: %d\n", rc);
        return rc;
    }
    
    printf("BLE: Scanner started, address: %s\n", addr_str(addr_val));
    
    // Start scanning
    printf("BLE: Starting scan...\n");
    start_scan();
    
    return 0;
}

// Application callback for BLE host reset
static void ble_app_on_reset(int reason)
{
    printf("BLE reset: %d\n", reason);
}

// Application callbacks
static void ble_app_on_sync_cb(void) { ble_app_on_sync(); }
static void ble_app_on_reset_cb(int reason) { ble_app_on_reset(reason); }

void app_main(void)
{
    printf("App: Starting...\n");
    
    // Initialize NVS
    printf("App: Initializing NVS...\n");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("App: NVS needs cleanup, erasing...\n");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    printf("App: NVS init status: %s\n", esp_err_to_name(ret));
    
    // Initialize BLE controller and NimBLE host
    printf("App: Initializing BLE...\n");
    esp_nimble_hci_init();
    
    printf("App: Initializing NimBLE port...\n");
    nimble_port_init();
    
    // Set the default device name
    printf("App: Setting device name...\n");
    ble_svc_gap_device_name_set("ESP32-BLE-Scanner");
    
    // Set callbacks
    printf("App: Setting up callbacks...\n");
    ble_hs_cfg.sync_cb = ble_app_on_sync_cb;
    ble_hs_cfg.reset_cb = ble_app_on_reset_cb;
    
    // Set the device's address type
    printf("App: Setting address type...\n");
    own_addr_type = BLE_OWN_ADDR_PUBLIC;
    
    // Disable security for scanning
    printf("App: Configuring security...\n");
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist = 0;
    ble_hs_cfg.sm_their_key_dist = 0;
    
    // Start the host task
    printf("App: Starting BLE host task...\n");
    nimble_port_freertos_init(ble_host_task);
    
    // Keep the main task alive
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
