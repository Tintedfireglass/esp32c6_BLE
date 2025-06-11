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

#include "driver/spi_master.h"

// LCD Function Prototypes
void ili9341_fill(uint16_t color);
void ili9341_text(const char *str, uint16_t x, uint16_t y, uint16_t color);
void ili9341_text_small(const char *str, uint16_t x, uint16_t y, uint16_t color);
void ili9341_text_medium(const char *str, uint16_t x, uint16_t y, uint16_t color);
void ili9341_text_large(const char *str, uint16_t x, uint16_t y, uint16_t color);
void ili9341_text_xlarge(const char *str, uint16_t x, uint16_t y, uint16_t color);
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Forward declarations
static void start_scan(void);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void rescan_task(void *arg);
static int discover_services(uint16_t conn_handle);
static void print_uuid(const ble_uuid_any_t *uuid);
static const char *chr_props_to_str(uint8_t props);
static int read_characteristic(uint16_t conn_handle, uint16_t val_handle);
static int subscribe_to_notifications(uint16_t conn_handle, uint16_t val_handle, uint16_t ccc_handle);
static int on_notify(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg);
static void periodic_read_task(void *arg);

// Target device address to connect to
static const uint8_t TARGET_ADDR[6] = {0xa6, 0x32, 0x0e, 0xe3, 0x85, 0xa0}; // a0:85:e3:0e:32:a6 in little-endian
static bool device_connected = false;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

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

// Function to connect to a BLE device
static void connect_to_device(const ble_addr_t *addr) {
    printf("Attempting to connect to %s...\n", addr_str(addr->val));
    
    // First, stop any ongoing scan
    int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        printf("Error stopping scan: %d\n", rc);
        return;
    }
    
    // Small delay to ensure scan is fully stopped
    vTaskDelay(pdMS_TO_TICKS(100));
    
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x60,
        .scan_window = 0x30,
        .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,
        .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,
        .latency = 0,
        .supervision_timeout = 0x0100,
        .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
        .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN
    };
    
    // Try to connect
    rc = ble_gap_connect(own_addr_type, addr, 30000, &conn_params, 
                        gap_event_cb, NULL);
    if (rc != 0) {
        printf("Error: Failed to connect to device: %d. Will retry...\n", rc);
        // Schedule a rescan after a delay using a static function
        static TaskHandle_t rescan_task_handle = NULL;
        if (rescan_task_handle == NULL) {  // Only create if not already running
            xTaskCreatePinnedToCore(
                rescan_task,
                "rescan_task",
                2048,
                NULL,
                5,
                &rescan_task_handle,
                0
            );
        }
        return;
    }
    
    printf("Connection initiated...\n");
}

// Called when an advertisement is received
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    // Remove unused variable
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
        
        // Check if this is our target device
        if (memcmp(event->disc.addr.val, TARGET_ADDR, 6) == 0 && !device_connected) {
            printf("Target device found! Attempting to connect...\n");
            connect_to_device(&event->disc.addr);
            // Don't set device_connected yet - wait for connection success
        }
        break;
        
    case BLE_GAP_EVENT_CONNECT:
        // A new connection was established or a connection attempt failed
        if (event->connect.status == 0) {
            // Connection successful
            printf("Connection established. Connection handle: %d\n", event->connect.conn_handle);
            conn_handle = event->connect.conn_handle;
            device_connected = true; // Only set this on successful connection
            // Show connected message on LCD
            ili9341_fill(0x0000);
            ili9341_text_medium("Rider Helmet Detected", 30, 120, 0xFFE0);
            // Start service discovery
            printf("Starting service discovery...\n");
            int rc = discover_services(conn_handle);
            if (rc != 0) {
                printf("Failed to start service discovery: %d\n", rc);
            }
        } else {
            // Connection attempt failed
            printf("Error: Connection failed, status: %d\n", event->connect.status);
            device_connected = false; // Allow reconnection attempt
            // Restart scanning after a short delay
            // Show searching message on LCD
ili9341_fill(0x0000);
ili9341_text_medium("Looking for helmet", 30, 120, 0xFFE0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            start_scan();
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        // Handle disconnection
        printf("Disconnected. Reason: %d\n", event->disconnect.reason);
        device_connected = false;
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        // Restart scanning after a short delay
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Show searching message on LCD
        ili9341_fill(0x0000);
        ili9341_text_medium("Looking for helmet", 30, 120, 0xFFE0);
        start_scan();
        break;
        
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (!device_connected) {
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
        }
        break;
        
    default:
        break;
    }
    
    return 0;
}

// Convert BLE UUID to string
static void print_uuid(const ble_uuid_any_t *uuid) {
    switch (uuid->u.type) {
        case BLE_UUID_TYPE_16:
            printf("0x%04" PRIx16, BLE_UUID16(uuid)->value);
            break;
        case BLE_UUID_TYPE_32:
            printf("0x%08" PRIx32, BLE_UUID32(uuid)->value);
            break;
        case BLE_UUID_TYPE_128:
            printf("0x%02" PRIx8 "%02" PRIx8 "...%02" PRIx8 "%02" PRIx8,
                   uuid->u128.value[15], uuid->u128.value[14],
                   uuid->u128.value[1], uuid->u128.value[0]);
            break;
        default:
            printf("(unknown type %d)", uuid->u.type);
            break;
    }
}

// Convert characteristic properties to string
static const char *chr_props_to_str(uint8_t props) {
    static char str[100] = {0};
    str[0] = '[';
    int pos = 1;
    
    if (props & BLE_GATT_CHR_PROP_READ) { strcpy(str + pos, " READ"); pos += 5; }
    if (props & BLE_GATT_CHR_PROP_WRITE) { strcpy(str + pos, " WRITE"); pos += 6; }
    if (props & BLE_GATT_CHR_PROP_WRITE_NO_RSP) { strcpy(str + pos, " WRITE_NR"); pos += 9; }
    if (props & BLE_GATT_CHR_PROP_NOTIFY) { strcpy(str + pos, " NOTIFY"); pos += 7; }
    if (props & BLE_GATT_CHR_PROP_INDICATE) { strcpy(str + pos, " INDICATE"); pos += 10; }
    
    if (pos == 1) {
        strcpy(str, "[NONE]");
    } else {
        str[pos] = ' ';
        str[pos+1] = ']';
        str[pos+2] = '\0';
    }
    
    return str;
}

// Notification/Indication callback
static int on_notify(uint16_t conn_handle, const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr, void *arg) {
    if (error != NULL) {
        if (error->status != 0) {
            printf("Notification error: %d\n", error->status);
            return error->status;
        }
    }

    if (attr == NULL) {
        printf("Notification received: NULL attribute\n");
        return 0;
    }

    printf("Notification received (handle=0x%04x): ", attr->handle);
    
    if (attr->om != NULL) {
        // Print all bytes
        for (int i = 0; i < attr->om->om_len && i < 32; i++) {  // Limit to first 32 bytes
            printf("%02x ", attr->om->om_data[i]);
        }
        printf("(%d bytes)\n", attr->om->om_len);
        
        // Check if this is the 0x0022 handle and has at least 1 byte of data
        if (attr->handle == 0x0022 && attr->om->om_len >= 1) {
            uint8_t first_byte = attr->om->om_data[0];
            printf("First byte (decimal): %u\n", first_byte);
            
            if (first_byte < 40) {
                printf("ALCOHOL DETECTED! (Value: %u < 40)\n", first_byte);
                // Show red warning at the bottom of the screen
                ili9341_text_small("WARNING ALCOHOL DETECTED", 40, 200, 0xF800); // 0xF800 is red
            } else {
                printf("No alcohol detected (Value: %u >= 40)\n", first_byte);
                // Clear the warning if it was previously shown
                ili9341_text_small("WARNING ALCOHOL DETECTED", 40, 200, 0x0000); // 0xF800 is red
            }
        }
    } else {
        printf("No data\n");
    }
    
    return 0;
}

// Read characteristic value
static int read_characteristic(uint16_t conn_handle, uint16_t val_handle) {
    printf("Reading characteristic value from handle 0x%04x...\n", val_handle);
    int rc = ble_gattc_read(conn_handle, val_handle, on_notify, NULL);
    if (rc != 0) {
        printf("Failed to read characteristic: %d\n", rc);
    }
    return rc;
}


// Subscribe to notifications
static int subscribe_to_notifications(uint16_t conn_handle, uint16_t val_handle, uint16_t ccc_handle) {
    printf("Subscribing to notifications for handle 0x%04x (CCCD: 0x%04x)...\n", val_handle, ccc_handle);
    
    // Write to CCCD to enable notifications (0x0001) or indications (0x0002)
    uint16_t cccd_val = 0x0001;  // 0x0001 for notifications, 0x0002 for indications
    int rc = ble_gattc_write_flat(conn_handle, ccc_handle, &cccd_val, sizeof(cccd_val), on_notify, NULL);
    if (rc != 0) {
        printf("Failed to write to CCCD: %d\n", rc);
        return rc;
    }
    
    return 0;
}

// Service discovery callback
static int disc_svc_chrs_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg) {
    (void)arg;  // Unused parameter
    
    if (error->status == BLE_HS_EDONE) {
        printf("  Characteristics discovery complete\n");
        return 0;
    }
    
    if (error->status != 0) {
        printf("  Characteristic discovery failed: %d\n", error->status);
        return error->status;
    }
    
    printf("  Characteristic: handle=0x%04x, def_handle=0x%04x, val_handle=0x%04x, props=",
           chr->val_handle - 1, chr->def_handle, chr->val_handle);
    printf("%s\n", chr_props_to_str(chr->properties));
    
    printf("    UUID: ");
    print_uuid((const ble_uuid_any_t *)&chr->uuid);
    printf("\n");
    
    // Check if this is from the last service (0x4444...0000)
    if (chr->def_handle >= 0x0020) {
        printf("\n=== Found target characteristic in last service ===\n");
        printf("Handle: 0x%04x, Properties: %s\n", chr->val_handle, chr_props_to_str(chr->properties));
          

        // If it supports READ, read its value
        if (chr->properties & BLE_GATT_CHR_PROP_READ) {
            read_characteristic(conn_handle, chr->val_handle);
        }
        
        // If it supports NOTIFY, subscribe to notifications
        if (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) {
            // CCCD is typically at val_handle + 1 for notifications
            subscribe_to_notifications(conn_handle, chr->val_handle, chr->val_handle + 1);
        }
    }
    
    return 0;
}

// Service discovery callback
static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                      const struct ble_gatt_svc *service, void *arg) {
    if (error->status == BLE_HS_EDONE) {
        printf("Service discovery complete\n");
        return 0;
    }
    
    if (error->status != 0) {
        printf("Service discovery failed: %d\n", error->status);
        return error->status;
    }
    
    printf("\nService found: start_handle=0x%04x, end_handle=0x%04x\n",
           service->start_handle, service->end_handle);
    
    printf("  UUID: ");
    print_uuid((const ble_uuid_any_t *)&service->uuid);
    printf("\n");
    
    // Discover characteristics for this service
    int rc = ble_gattc_disc_all_chrs(conn_handle, service->start_handle, 
                                    service->end_handle, disc_svc_chrs_cb, NULL);
    if (rc != 0) {
        printf("Failed to discover characteristics: %d\n", rc);
        return rc;
    }
    
    return 0;
}

// Start service discovery
static int discover_services(uint16_t conn_handle) {
    printf("Discovering services...\n");
    
    // Start discovering all services
    int rc = ble_gattc_disc_all_svcs(conn_handle, disc_svc_cb, NULL);
    if (rc != 0) {
        printf("Failed to start service discovery: %d\n", rc);
        return rc;
    }
    
    return 0;
}

// Task to handle rescan after delay
static void rescan_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    start_scan();
    vTaskDelete(NULL);
}

// Task to periodically read the characteristic
static void periodic_read_task(void *arg) {
    while (1) {
        if (device_connected && conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            printf("\n[Periodic Read] ");
            read_characteristic(conn_handle, 0x0022);  // Read handle 0x0022
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);  // 3 second delay
    }
}

// Start BLE scanning
void start_scan(void)
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
    ili9341_fill(0x0000); 
    ili9341_text_medium("Searching for Helmet", 30, 120, 0xFFFF);
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


// ==== Pin Definitions ====
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK  7
#define PIN_NUM_CS   46
#define PIN_NUM_DC   5
#define PIN_NUM_RST  4
#define PIN_NUM_BCKL 15

#define SPI_HOST SPI2_HOST
#define LCD_WIDTH  320
#define LCD_HEIGHT 240

static spi_device_handle_t spi;

// ==== 5x8 ASCII Font Table (32-127) ====
static const uint8_t font5x8[96][5] = {
    // ASCII 32-127
    {0x00,0x00,0x00,0x00,0x00}, // 32  ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 33  '!'
    {0x00,0x07,0x00,0x07,0x00}, // 34  '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 35  '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36  '$'
    {0x23,0x13,0x08,0x64,0x62}, // 37  '%'
    {0x36,0x49,0x55,0x22,0x50}, // 38  '&'
    {0x00,0x05,0x03,0x00,0x00}, // 39  '''
    {0x00,0x1C,0x22,0x41,0x00}, // 40  '('
    {0x00,0x41,0x22,0x1C,0x00}, // 41  ')'
    {0x14,0x08,0x3E,0x08,0x14}, // 42  '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 43  '+'
    {0x00,0x50,0x30,0x00,0x00}, // 44  ','
    {0x08,0x08,0x08,0x08,0x08}, // 45  '-'
    {0x00,0x60,0x60,0x00,0x00}, // 46  '.'
    {0x20,0x10,0x08,0x04,0x02}, // 47  '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 48  '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 49  '1'
    {0x42,0x61,0x51,0x49,0x46}, // 50  '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 51  '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 52  '4'
    {0x27,0x45,0x45,0x45,0x39}, // 53  '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 54  '6'
    {0x01,0x71,0x09,0x05,0x03}, // 55  '7'
    {0x36,0x49,0x49,0x49,0x36}, // 56  '8'
    {0x06,0x49,0x49,0x29,0x1E}, // 57  '9'
    {0x00,0x36,0x36,0x00,0x00}, // 58  ':'
    {0x00,0x56,0x36,0x00,0x00}, // 59  ';'
    {0x08,0x14,0x22,0x41,0x00}, // 60  '<'
    {0x14,0x14,0x14,0x14,0x14}, // 61  '='
    {0x00,0x41,0x22,0x14,0x08}, // 62  '>'
    {0x02,0x01,0x51,0x09,0x06}, // 63  '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 64  '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 65  'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 66  'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 67  'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 68  'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 69  'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 70  'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 71  'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 72  'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 73  'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 74  'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 75  'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 76  'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 77  'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 78  'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 79  'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 80  'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 81  'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 82  'R'
    {0x46,0x49,0x49,0x49,0x31}, // 83  'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 84  'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 85  'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 86  'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 87  'W'
    {0x63,0x14,0x08,0x14,0x63}, // 88  'X'
    {0x07,0x08,0x70,0x08,0x07}, // 89  'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 90  'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // 91  '['
    {0x02,0x04,0x08,0x10,0x20}, // 92  '\'
    {0x00,0x41,0x41,0x7F,0x00}, // 93  ']'
    {0x04,0x02,0x01,0x02,0x04}, // 94  '^'
    {0x40,0x40,0x40,0x40,0x40}, // 95  '_'
    {0x00,0x01,0x02,0x04,0x00}, // 96  '`'
    {0x20,0x54,0x54,0x54,0x78}, // 97  'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 98  'b'
    {0x38,0x44,0x44,0x44,0x20}, // 99  'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 100 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 101 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 102 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 103 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 104 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 105 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 106 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 107 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 108 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 109 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 110 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 111 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 112 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 113 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 114 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 115 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 116 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 117 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 118 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 119 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 120 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 121 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 122 'z'
    {0x00,0x08,0x36,0x41,0x00}, // 123 '{'
    {0x00,0x00,0x7F,0x00,0x00}, // 124 '|'
    {0x00,0x41,0x36,0x08,0x00}, // 125 '}'
    {0x10,0x08,0x08,0x10,0x08}, // 126 '~'
    {0x00,0x00,0x00,0x00,0x00}  // 127 DEL
};

// ==== Utility ====
static inline void delay_ms(int ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }

// ==== GPIO Init ====
static void ili9341_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST) | 
                        (1ULL << PIN_NUM_BCKL) | (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_level(PIN_NUM_DC, 0);
    gpio_set_level(PIN_NUM_RST, 1);
    if (PIN_NUM_BCKL != -1) gpio_set_level(PIN_NUM_BCKL, 1);
}

// ==== SPI Init ====
static void spi_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2 + 8
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1, // CS handled manually
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));
}

// ==== LCD Command/Data ====
static void ili9341_write_cmd(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0);
    gpio_set_level(PIN_NUM_CS, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    gpio_set_level(PIN_NUM_CS, 1);
}
static void ili9341_write_data(const uint8_t* data, int len) {
    gpio_set_level(PIN_NUM_DC, 1);
    gpio_set_level(PIN_NUM_CS, 0);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    gpio_set_level(PIN_NUM_CS, 1);
}

// ==== LCD Reset ====
static void ili9341_reset(void) {
    gpio_set_level(PIN_NUM_RST, 1); delay_ms(50);
    gpio_set_level(PIN_NUM_RST, 0); delay_ms(50);
    gpio_set_level(PIN_NUM_RST, 1); delay_ms(150);
}

// ==== LCD Init Sequence ====
static void ili9341_init(void) {
    ili9341_reset();
    ili9341_write_cmd(0x01); delay_ms(100);
    ili9341_write_cmd(0x28);
    ili9341_write_cmd(0xCF); uint8_t cf[]={0x00,0x83,0x30}; ili9341_write_data(cf,3);
    ili9341_write_cmd(0xED); uint8_t ed[]={0x64,0x03,0x12,0x81}; ili9341_write_data(ed,4);
    ili9341_write_cmd(0xE8); uint8_t e8[]={0x85,0x01,0x79}; ili9341_write_data(e8,3);
    ili9341_write_cmd(0xCB); uint8_t cb[]={0x39,0x2C,0x00,0x34,0x02}; ili9341_write_data(cb,5);
    ili9341_write_cmd(0xF7); uint8_t f7[]={0x20}; ili9341_write_data(f7,1);
    ili9341_write_cmd(0xEA); uint8_t ea[]={0x00,0x00}; ili9341_write_data(ea,2);
    ili9341_write_cmd(0xC0); uint8_t c0[]={0x26}; ili9341_write_data(c0,1);
    ili9341_write_cmd(0xC1); uint8_t c1[]={0x11}; ili9341_write_data(c1,1);
    ili9341_write_cmd(0xC5); uint8_t c5[]={0x35,0x3E}; ili9341_write_data(c5,2);
    ili9341_write_cmd(0xC7); uint8_t c7[]={0xBE}; ili9341_write_data(c7,1);
    ili9341_write_cmd(0x36); uint8_t d36[]={0x28}; ili9341_write_data(d36,1);
    ili9341_write_cmd(0x3A); uint8_t d3a[]={0x55}; ili9341_write_data(d3a,1);
    ili9341_write_cmd(0xB1); uint8_t b1[]={0x00,0x1B}; ili9341_write_data(b1,2);
    ili9341_write_cmd(0xF2); uint8_t f2[]={0x08}; ili9341_write_data(f2,1);
    ili9341_write_cmd(0x26); uint8_t d26[]={0x01}; ili9341_write_data(d26,1);
    ili9341_write_cmd(0xE0); uint8_t e0[]={0x1F,0x1A,0x18,0x0A,0x0F,0x06,0x45,0x87,0x32,0x0A,0x07,0x02,0x07,0x05,0x00}; ili9341_write_data(e0,15);
    ili9341_write_cmd(0xE1); uint8_t e1[]={0x00,0x25,0x27,0x05,0x10,0x09,0x3A,0x78,0x4D,0x05,0x18,0x0D,0x38,0x3A,0x1F}; ili9341_write_data(e1,15);
    ili9341_write_cmd(0x11); delay_ms(120);
    ili9341_write_cmd(0x29);
}

// ==== Set Drawing Window ====
static void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    ili9341_write_cmd(0x2A);
    uint8_t data_col[] = {x0>>8, x0&0xFF, x1>>8, x1&0xFF};
    ili9341_write_data(data_col, 4);
    ili9341_write_cmd(0x2B);
    uint8_t data_row[] = {y0>>8, y0&0xFF, y1>>8, y1&0xFF};
    ili9341_write_data(data_row, 4);
    ili9341_write_cmd(0x2C);
}

// ==== Fill Screen ====
void ili9341_fill(uint16_t color) {
    ili9341_set_window(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    uint8_t buf[128];
    for (int i = 0; i < sizeof(buf)/2; i++) { buf[2*i]=hi; buf[2*i+1]=lo; }
    gpio_set_level(PIN_NUM_DC, 1);
    gpio_set_level(PIN_NUM_CS, 0);
    int pixels = LCD_WIDTH * LCD_HEIGHT;
    while (pixels > 0) {
        int to_send = pixels > (sizeof(buf)/2) ? (sizeof(buf)/2) : pixels;
        spi_transaction_t t = { .length = to_send*16, .tx_buffer = buf };
        ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
        pixels -= to_send;
    }
    gpio_set_level(PIN_NUM_CS, 1);
}

// ==== Draw Single Character ====
static void ili9341_draw_char(char c, uint16_t x, uint16_t y, uint16_t color) {
    if (c < 32 || c > 127) c = '?';
    const uint8_t *bitmap = font5x8[c-32];
    ili9341_set_window(x, y, x+4, y+7);
    gpio_set_level(PIN_NUM_DC, 1);
    gpio_set_level(PIN_NUM_CS, 0);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    uint8_t buf[2];
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 5; col++) {
            if (bitmap[col] & (1 << row)) { buf[0]=hi; buf[1]=lo; }
            else { buf[0]=0; buf[1]=0; }
            spi_transaction_t t = { .length = 16, .tx_buffer = buf };
            ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
        }
    }
    gpio_set_level(PIN_NUM_CS, 1);
}

// ==== Draw Text String ====
void ili9341_text(const char *str, uint16_t x, uint16_t y, uint16_t color) {
    while (*str) {
        ili9341_draw_char(*str, x, y, color);
        x += 6; // 5px font + 1px space
        str++;
    }
}
// ==== Draw Single Character with Scale ====
static void ili9341_draw_char_scaled(char c, uint16_t x, uint16_t y, uint16_t color, uint8_t scale) {
    if (c < 32 || c > 127) c = '?';
    if (scale == 0) scale = 1; // Prevent division by zero
    
    const uint8_t *bitmap = font5x8[c-32];
    
    // Set window for scaled character
    ili9341_set_window(x, y, x + (5 * scale) - 1, y + (8 * scale) - 1);
    
    gpio_set_level(PIN_NUM_DC, 1);
    gpio_set_level(PIN_NUM_CS, 0);
    
    uint8_t hi = color >> 8, lo = color & 0xFF;
    uint8_t buf[2];
    
    // Draw each row of the character
    for (int row = 0; row < 8; row++) {
        // Repeat each row 'scale' times for vertical scaling
        for (int row_repeat = 0; row_repeat < scale; row_repeat++) {
            // Draw each column of the character
            for (int col = 0; col < 5; col++) {
                // Check if pixel should be on
                bool pixel_on = bitmap[col] & (1 << row);
                
                // Repeat each pixel 'scale' times for horizontal scaling
                for (int col_repeat = 0; col_repeat < scale; col_repeat++) {
                    if (pixel_on) { 
                        buf[0] = hi; 
                        buf[1] = lo; 
                    } else { 
                        buf[0] = 0; 
                        buf[1] = 0; 
                    }
                    spi_transaction_t t = { .length = 16, .tx_buffer = buf };
                    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
                }
            }
        }
    }
    gpio_set_level(PIN_NUM_CS, 1);
}

// ==== Draw Text String with Scale ====
static void ili9341_text_scaled(const char *str, uint16_t x, uint16_t y, uint16_t color, uint8_t scale) {
    if (scale == 0) scale = 1;
    
    uint16_t char_width = 5 * scale + scale; // scaled font width + spacing
    uint16_t current_x = x;
    
    while (*str) {
        ili9341_draw_char_scaled(*str, current_x, y, color, scale);
        current_x += char_width;
        str++;
    }
}

// ==== Convenience functions for common sizes ====
void ili9341_text_small(const char *str, uint16_t x, uint16_t y, uint16_t color) {
    ili9341_text_scaled(str, x, y, color, 1); // 5x8 pixels
}

void ili9341_text_medium(const char *str, uint16_t x, uint16_t y, uint16_t color) {
    ili9341_text_scaled(str, x, y, color, 2); // 10x16 pixels
}

void ili9341_text_large(const char *str, uint16_t x, uint16_t y, uint16_t color) {
    ili9341_text_scaled(str, x, y, color, 3); // 15x24 pixels
}

void ili9341_text_xlarge(const char *str, uint16_t x, uint16_t y, uint16_t color) {
    ili9341_text_scaled(str, x, y, color, 4); // 20x32 pixels
}

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
    
    //init LCD
    ili9341_gpio_init();
    spi_init();
    ili9341_init();

    ili9341_fill(0x0000); // Black background

    ili9341_text_medium("WELCOME", 40, 100, 0xFFFF);        // 2x size (10x16 pixels)
    ili9341_text_small("DEVICE STARTING...", 20, 130, 0x07E0); // 1x size (5x8 pixels)
    
    if (PIN_NUM_BCKL != -1) gpio_set_level(PIN_NUM_BCKL, 1);
    // Initialize BLE controller and NimBLE host

    vTaskDelay(pdMS_TO_TICKS(1000)); // one second delay
    ili9341_fill(0x0000); 
    printf("App: Initializing BLE...\n");
    ili9341_fill(0x0000); 
    ili9341_text_medium("Initializing BLE", 30, 120, 0xFFFF);        // 3x size (15x24 pixels)
    
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
    
    // Create a task for periodic reads
    xTaskCreate(periodic_read_task, "periodic_read", 4096, NULL, 5, NULL);
    
    // Keep the main task alive
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}