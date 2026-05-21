idf_component_register(
    SRCS "micMain.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_adc esp_driver_gpio
)