#pragma once

#include "ble_remote.h"
#include "ch224a.h"
#include "dispenser.h"
#include "driver/i2c_master.h"
#include "drv5055.h"
#include "drv8871.h"
#include "esp_err.h"
#include "gc9a01a.h"
#include "max98357a.h"
#include "rv3028.h"
#include "scheduler.h"
#include "screen.h"
#include "vsense.h"

/**
 * Initialize the ESP-IDF console REPL and register application commands.
 *
 * @param i2c_bus Shared I2C master bus, used by the i2c_scan command.
 * @param motor_handle Initialized DRV8871 handle.
 * @param audio_handle Initialized MAX98357A handle.
 * @param rtc_handle Initialized RV-3028 handle, NULL when the RTC is absent.
 * @param pd_handle Initialized CH224A handle, NULL when the chip is absent.
 * @param hall_handle Initialized DRV5055 handle.
 * @param supply_handle Initialized +9V rail monitor handle.
 * @param drum_handle Initialized dispenser motion handle.
 * @param schedule_handle Dispensing schedule handle, NULL when the RTC is absent.
 * @param ble_handle BLE peripheral handle, NULL when the radio failed to start.
 * @param display_handle GC9A01A panel handle, NULL when the display failed to start.
 * @param screen_handle Clock face handle, NULL when the display failed to start.
 * @return ESP_OK on success or an error from esp_console_* APIs.
 */
esp_err_t console_start(i2c_master_bus_handle_t i2c_bus, drv8871_handle_t motor_handle, max98357a_handle_t audio_handle,
                        rv3028_handle_t rtc_handle, ch224a_handle_t pd_handle, drv5055_handle_t hall_handle,
                        vsense_handle_t supply_handle, dispenser_handle_t drum_handle,
                        scheduler_handle_t schedule_handle, ble_remote_handle_t ble_handle,
                        gc9a01a_handle_t display_handle, screen_handle_t screen_handle);
