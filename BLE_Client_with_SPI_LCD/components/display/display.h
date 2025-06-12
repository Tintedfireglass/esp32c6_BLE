#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==== Configuration Constants ====
#define ILI9341_WIDTH  320
#define ILI9341_HEIGHT 240

// ==== Color Definitions ====
#define ILI9341_BLACK   0x0000
#define ILI9341_WHITE   0xFFFF
#define ILI9341_RED     0xF800
#define ILI9341_GREEN   0x07E0
#define ILI9341_BLUE    0x001F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F

// ==== Configuration Structure ====
typedef struct {
    // SPI Configuration
    spi_host_device_t spi_host;
    int pin_miso;
    int pin_mosi;
    int pin_clk;
    int pin_cs;
    
    // Control Pins
    int pin_dc;
    int pin_rst;
    int pin_bckl;
    
    // SPI Speed
    int spi_clock_speed_hz;
} ili9341_config_t;

// ==== Public Function Declarations ====

/**
 * @brief Initialize the ILI9341 display
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ili9341_init(const ili9341_config_t *config);

/**
 * @brief Deinitialize the ILI9341 display and free resources
 */
void ili9341_deinit(void);

/**
 * @brief Fill the entire screen with a color
 * @param color 16-bit RGB565 color value
 */
void ili9341_fill(uint16_t color);

/**
 * @brief Set the drawing window (region of interest)
 * @param x0 Starting X coordinate
 * @param y0 Starting Y coordinate
 * @param x1 Ending X coordinate
 * @param y1 Ending Y coordinate
 */
void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief Draw a single pixel
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color value
 */
void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw text at normal size (5x8 pixels per character)
 * @param str String to display
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color value
 */
void ili9341_text(const char *str, uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw text at small size (5x8 pixels per character)
 * @param str String to display
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color value
 */
void ili9341_text_small(const char *str, uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw text at medium size (10x16 pixels per character)
 * @param str String to display
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color value
 */
void ili9341_text_medium(const char *str, uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw text at large size (15x24 pixels per character)
 * @param str String to display
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color value
 */
void ili9341_text_large(const char *str, uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw text at extra large size (20x32 pixels per character)
 * @param str String to display
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color value
 */
void ili9341_text_xlarge(const char *str, uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Set backlight state
 * @param state true to turn on, false to turn off
 */
void ili9341_set_backlight(bool state);

/**
 * @brief Convert RGB values to RGB565 color
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return 16-bit RGB565 color value
 */
uint16_t ili9341_color_rgb(uint8_t r, uint8_t g, uint8_t b);
#ifdef __cplusplus
}
#endif

#endif // DISPLAY_H
