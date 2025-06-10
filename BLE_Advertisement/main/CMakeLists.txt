idf_component_register(
    SRCS "helmetESP.c"
    REQUIRES 
        nvs_flash
        esp_system
        freertos
        driver
        esp_adc
        bt
)
