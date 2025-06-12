#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>

static const char *TAG = "ILI9341";

// ==== Private Variables ====
static spi_device_handle_t spi_device = NULL;
static const ili9341_config_t *display_config = NULL;
static bool is_initialized = false;

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

// ==== Private Function Declarations ====

static void ili9341_write_cmd(uint8_t cmd);
static void ili9341_write_data(const uint8_t* data, int len);
static void ili9341_reset(void);
static void ili9341_hw_init(void);
static void ili9341_draw_char(char c, uint16_t x, uint16_t y, uint16_t color);
void ili9341_draw_char_scaled(char c, uint16_t x, uint16_t y, uint16_t color, uint8_t scale);
void ili9341_text_scaled(const char *str, uint16_t x, uint16_t y, uint16_t color, uint8_t scale);
static inline void delay_ms(int ms);

// ==== Private Functions ====
static inline void delay_ms(int ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static esp_err_t ili9341_gpio_init(void) {
    if (display_config == NULL) {
        ESP_LOGE(TAG, "Display config not set");
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << display_config->pin_dc) | 
                       (1ULL << display_config->pin_rst) | 
                       (1ULL << display_config->pin_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    // Add backlight pin if configured
    if (display_config->pin_bckl >= 0) {
        io_conf.pin_bit_mask |= (1ULL << display_config->pin_bckl);
    }
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set initial pin states
    gpio_set_level(display_config->pin_cs, 1);
    gpio_set_level(display_config->pin_dc, 0);
    gpio_set_level(display_config->pin_rst, 1);
    
    if (display_config->pin_bckl >= 0) {
        gpio_set_level(display_config->pin_bckl, 1);
    }
    
    return ESP_OK;
}

esp_err_t ili9341_init(const ili9341_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Store the config pointer
    display_config = config;
    
    // Initialize GPIO
    esp_err_t ret = ili9341_gpio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO init failed");
        return ret;
    }
    
    // Initialize SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = display_config->pin_miso,
        .mosi_io_num = display_config->pin_mosi,
        .sclk_io_num = display_config->pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ILI9341_WIDTH * ILI9341_HEIGHT * 2 + 8
    };
    
    // Initialize SPI bus
    ret = spi_bus_initialize(display_config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device - Updated to match example
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000, // Set to 40MHz like example
        .mode = 0,
        .spics_io_num = -1, // CS handled manually
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    
    // Add SPI device
    ret = spi_bus_add_device(display_config->spi_host, &devcfg, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(display_config->spi_host);
        return ret;
    }
    
    // Initialize display hardware
    ili9341_hw_init();
    is_initialized = true;
    
    ESP_LOGI(TAG, "Display initialized successfully");
    return ESP_OK;
}

void ili9341_deinit(void) {
    if (!is_initialized) {
        return; // Already deinitialized
    }
    
    // Turn off display backlight if configured
    if (display_config && display_config->pin_bckl >= 0) {
        gpio_set_level(display_config->pin_bckl, 0);
    }
    
    // Remove SPI device if it was added
    if (spi_device) {
        spi_bus_remove_device(spi_device);
        spi_device = NULL;
    }
    
    // Free SPI bus if it was initialized
    if (display_config) {
        spi_bus_free(display_config->spi_host);
    }
    
    // Reset state
    is_initialized = false;
    display_config = NULL;
    
    ESP_LOGI(TAG, "Display deinitialized");
}

static void ili9341_write_cmd(uint8_t cmd) {
    if (display_config == NULL) return;
    gpio_set_level(display_config->pin_dc, 0);
    gpio_set_level(display_config->pin_cs, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    if (spi_device) {
        ESP_ERROR_CHECK(spi_device_transmit(spi_device, &t));
    }
    gpio_set_level(display_config->pin_cs, 1);
}

static void ili9341_write_data(const uint8_t* data, int len) {
    if (display_config == NULL || data == NULL || len <= 0) return;
    
    gpio_set_level(display_config->pin_dc, 1);
    gpio_set_level(display_config->pin_cs, 0);
    
    spi_transaction_t t = { 
        .length = len * 8, 
        .tx_buffer = data 
    };
    
    if (spi_device) {
        esp_err_t ret = spi_device_transmit(spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        }
    }
    
    gpio_set_level(display_config->pin_cs, 1);
}

void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    if (display_config == NULL) return;
    
    // Column address set
    uint8_t col_data[4] = {
        (x0 >> 8) & 0xFF,
        x0 & 0xFF,
        (x1 >> 8) & 0xFF,
        x1 & 0xFF
    };
    
    // Page address set
    uint8_t row_data[4] = {
        (y0 >> 8) & 0xFF,
        y0 & 0xFF,
        (y1 >> 8) & 0xFF,
        y1 & 0xFF
    };
    
    // Send commands and data
    ili9341_write_cmd(0x2A); // Column address set
    ili9341_write_data(col_data, 4);
    
    ili9341_write_cmd(0x2B); // Page address set
    ili9341_write_data(row_data, 4);
    
    ili9341_write_cmd(0x2C); // Memory write
}

static void ili9341_reset(void) {
    if (display_config == NULL) return;
    
    ESP_LOGI(TAG, "Starting display reset sequence");
    
    gpio_set_level(display_config->pin_rst, 1); 
    delay_ms(100); // Increased delay
    gpio_set_level(display_config->pin_rst, 0); 
    delay_ms(100); // Increased delay
    gpio_set_level(display_config->pin_rst, 1); 
    delay_ms(200); // Increased delay
    
    ESP_LOGI(TAG, "Display reset sequence complete");
}

static void ili9341_hw_init(void) {
    if (display_config == NULL) {
        ESP_LOGE(TAG, "Display config not set in hw_init");
        return;
    }
    
    ESP_LOGI(TAG, "Starting display hardware initialization...");
    
    // Reset display
    ESP_LOGI(TAG, "Performing display reset...");
    ili9341_reset();
    
    // Send initialization commands
    const uint8_t *data;
    uint8_t cmd;
    
    ESP_LOGI(TAG, "Sending initialization commands...");
    
    // Software reset
    cmd = 0x01; 
    ESP_LOGI(TAG, "Sending software reset command");
    ili9341_write_cmd(cmd); 
    delay_ms(100);
    
    // Display off
    cmd = 0x28; 
    ESP_LOGI(TAG, "Turning display off");
    ili9341_write_cmd(cmd);
    
    // Power control B
    cmd = 0xCF; 
    ESP_LOGI(TAG, "Setting power control B");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x00, 0x83, 0x30}; 
    ili9341_write_data(data, 3);
    
    // Power on sequence control
    cmd = 0xED; 
    ESP_LOGI(TAG, "Setting power on sequence");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x64, 0x03, 0x12, 0x81}; 
    ili9341_write_data(data, 4);
    
    // Driver timing control A
    cmd = 0xE8; 
    ESP_LOGI(TAG, "Setting driver timing A");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x85, 0x01, 0x79}; 
    ili9341_write_data(data, 3);
    
    // Power control A
    cmd = 0xCB; 
    ESP_LOGI(TAG, "Setting power control A");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x39, 0x2C, 0x00, 0x34, 0x02}; 
    ili9341_write_data(data, 5);
    
    // Pump ratio control
    cmd = 0xF7; 
    ESP_LOGI(TAG, "Setting pump ratio");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x20}; 
    ili9341_write_data(data, 1);
    
    // Driver timing control B
    cmd = 0xEA; 
    ESP_LOGI(TAG, "Setting driver timing B");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x00, 0x00}; 
    ili9341_write_data(data, 2);
    
    // Power control 1
    cmd = 0xC0; 
    ESP_LOGI(TAG, "Setting power control 1");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x26}; 
    ili9341_write_data(data, 1);
    
    // Power control 2
    cmd = 0xC1; 
    ESP_LOGI(TAG, "Setting power control 2");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x11}; 
    ili9341_write_data(data, 1);
    
    // VCOM control 1
    cmd = 0xC5; 
    ESP_LOGI(TAG, "Setting VCOM control 1");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x35, 0x3E}; 
    ili9341_write_data(data, 2);
    
    // VCOM control 2
    cmd = 0xC7; 
    ESP_LOGI(TAG, "Setting VCOM control 2");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0xBE}; 
    ili9341_write_data(data, 1);
    
    // Memory access control - Changed to match example
    cmd = 0x36; 
    ESP_LOGI(TAG, "Setting memory access control");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x28}; // Changed to 0x28 for landscape orientation
    ili9341_write_data(data, 1);
    
    // Pixel format
    cmd = 0x3A; 
    ESP_LOGI(TAG, "Setting pixel format");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x55}; // 16-bit color
    ili9341_write_data(data, 1);
    
    // Frame rate control
    cmd = 0xB1; 
    ESP_LOGI(TAG, "Setting frame rate");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x00, 0x1B}; 
    ili9341_write_data(data, 2);
    
    // 3GAMMA DISABLE
    cmd = 0xF2; 
    ESP_LOGI(TAG, "Disabling 3GAMMA");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x08}; 
    ili9341_write_data(data, 1);
    
    // Gamma set
    cmd = 0x26; 
    ESP_LOGI(TAG, "Setting gamma");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x01}; 
    ili9341_write_data(data, 1);
    
    // Positive gamma correction
    cmd = 0xE0; 
    ESP_LOGI(TAG, "Setting positive gamma correction");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0x87, 
                       0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00};
    ili9341_write_data(data, 15);
    
    // Negative gamma correction
    cmd = 0xE1; 
    ESP_LOGI(TAG, "Setting negative gamma correction");
    ili9341_write_cmd(cmd);
    data = (uint8_t[]){0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 
                       0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F};
    ili9341_write_data(data, 15);
    
    // Exit sleep mode
    cmd = 0x11; 
    ESP_LOGI(TAG, "Exiting sleep mode");
    ili9341_write_cmd(cmd);
    delay_ms(120);
    
    // Turn on display
    cmd = 0x29; 
    ESP_LOGI(TAG, "Turning display on");
    ili9341_write_cmd(cmd);
    
    // Fill screen with black to ensure display is working
    ESP_LOGI(TAG, "Filling screen with black");
    ili9341_fill(0x0000);
    
    ESP_LOGI(TAG, "Display hardware initialization complete");
}

static void ili9341_draw_char(char c, uint16_t x, uint16_t y, uint16_t color) {
    if (display_config == NULL || c < 32 || c > 127) {
        c = '?';  // Replace with question mark if invalid character or config
    }
    
    const uint8_t *bitmap = font5x8[c-32];
    ili9341_set_window(x, y, x+4, y+7);
    
    if (display_config == NULL) return;
    
    gpio_set_level(display_config->pin_dc, 1);
    gpio_set_level(display_config->pin_cs, 0);
    
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    uint8_t buf[2];
    esp_err_t ret;
    
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 5; col++) {
            if (bitmap[col] & (1 << row)) {
                buf[0] = hi;
                buf[1] = lo;
            } else {
                buf[0] = 0;
                buf[1] = 0;
            }
            
            spi_transaction_t t = {
                .length = 16,
                .tx_buffer = buf
            };
            
            if (spi_device) {
                ret = spi_device_transmit(spi_device, &t);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
                }
            }
        }
    }
    gpio_set_level(display_config->pin_cs, 1);
}

void ili9341_draw_char_scaled(char c, uint16_t x, uint16_t y, uint16_t color, uint8_t scale) {
    if (display_config == NULL || scale == 0) {
        return;
    }
    
    if (c < 32 || c > 127) {
        c = '?'; // Replace with question mark for invalid characters
    }
    
    const uint8_t *bitmap = font5x8[c-32];
    
    // Set window for scaled character
    ili9341_set_window(x, y, x + (5 * scale) - 1, y + (8 * scale) - 1);
    
    gpio_set_level(display_config->pin_dc, 1);
    gpio_set_level(display_config->pin_cs, 0);
    
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    uint8_t buf[2];
    esp_err_t ret;
    
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
                    
                    if (spi_device) {
                        spi_transaction_t t = { 
                            .length = 16, 
                            .tx_buffer = buf 
                        };
                        ret = spi_device_transmit(spi_device, &t);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
                        }
                    }
                }
            }
        }
    }
    gpio_set_level(display_config->pin_cs, 1);
}

// ==== Draw Text String with Scale ====
void ili9341_text_scaled(const char *str, uint16_t x, uint16_t y, uint16_t color, uint8_t scale) {
    if (display_config == NULL || str == NULL || scale == 0) {
        return;
    }
    
    uint16_t char_width = 5 * scale + scale; // scaled font width + spacing
    uint16_t current_x = x;
    
    while (*str) {
        if (current_x > ILI9341_WIDTH) { // Prevent drawing outside screen
            break;
        }
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

void ili9341_fill(uint16_t color) {
    if (display_config == NULL) {
        ESP_LOGE(TAG, "Display not initialized");
        return;
    }

    // Set the entire display area
    ili9341_set_window(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
    
    // Prepare the color data (2 bytes per pixel)
    uint8_t color_hi = color >> 8;
    uint8_t color_lo = color & 0xFF;
    
    // Calculate the number of pixels to fill
    uint32_t total_pixels = (uint32_t)ILI9341_WIDTH * ILI9341_HEIGHT;
    
    // Fill in chunks to avoid large stack allocations
    const uint16_t chunk_size = 64; // Number of pixels to process at once
    uint8_t pixel_data[chunk_size * 2]; // 2 bytes per pixel
    
    // Fill the pixel data buffer with the specified color
    for (int i = 0; i < chunk_size; i++) {
        pixel_data[i * 2] = color_hi;
        pixel_data[i * 2 + 1] = color_lo;
    }
    
    // Send data in chunks
    uint32_t pixels_remaining = total_pixels;
    while (pixels_remaining > 0) {
        uint16_t pixels_to_send = (pixels_remaining > chunk_size) ? chunk_size : pixels_remaining;
        
        // Set DC high for data
        gpio_set_level(display_config->pin_dc, 1);
        gpio_set_level(display_config->pin_cs, 0);
        
        // Send pixel data
        if (spi_device) {
            spi_transaction_t t = {
                .length = pixels_to_send * 16, // 16 bits per pixel
                .tx_buffer = pixel_data
            };
            
            esp_err_t ret = spi_device_transmit(spi_device, &t);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
                break;
            }
        }
        
        // Update remaining pixels
        pixels_remaining -= pixels_to_send;
        
        // Deselect display
        gpio_set_level(display_config->pin_cs, 1);
    }
}