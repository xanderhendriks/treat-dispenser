idf_component_register(SRCS "console.c" "main.c"
                       REQUIRES ble_remote ch224a dispenser drv5055 drv8871 max98357a rv3028 scheduler vsense console esp_driver_i2c esp_adc nvs_flash
                       PRIV_REQUIRES esp_app_format
                       INCLUDE_DIRS "")
