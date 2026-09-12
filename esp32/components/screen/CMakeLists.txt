idf_component_register(SRCS "screen.c" "sevenseg.c" "pixelfont.c"
                           "assets/dog_badge.c" "assets/dog_large.c" "assets/bt_icon.c"
                      INCLUDE_DIRS "include"
                      PRIV_INCLUDE_DIRS "assets"
                      REQUIRES gc9a01a rv3028 scheduler ble_remote max98357a)
