idf_component_register(SRCS "console.c" "main.c"
                       REQUIRES ch224a drv5055 drv8871 max98357a rv3028 vsense console esp_driver_i2c esp_adc
                       PRIV_REQUIRES esp_app_format
                       INCLUDE_DIRS "")
