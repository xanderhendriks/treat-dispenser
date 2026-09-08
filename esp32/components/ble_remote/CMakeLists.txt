idf_component_register(SRCS "ble_remote.c"
                      INCLUDE_DIRS "include"
                      REQUIRES dispenser scheduler rv3028 bt esp_timer)
