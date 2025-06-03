cmake_minimum_required(VERSION 3.16)
set(IDF_TARGET esp32c6)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(helmetESP)

# Main components
set(COMPONENT_REQUIRES 
    bt
    nvs_flash
    esp_hw_support
    esp_system
    esp_bt
    esp_common
    esp_rom
    driver
    esp_event
    freertos
)
