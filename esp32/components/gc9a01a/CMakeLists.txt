idf_component_register(SRCS "gc9a01a.c" "gc9a01a_care.c"
                      INCLUDE_DIRS "include"
                      REQUIRES esp_driver_spi esp_driver_gpio esp_driver_ledc esp_timer)
