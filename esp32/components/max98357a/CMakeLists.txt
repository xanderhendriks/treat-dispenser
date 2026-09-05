idf_component_register(SRCS "max98357a.c" "melodies.c"
                      INCLUDE_DIRS "include"
                      PRIV_INCLUDE_DIRS "."
                      REQUIRES esp_driver_i2s esp_driver_gpio
                      PRIV_REQUIRES freertos)
