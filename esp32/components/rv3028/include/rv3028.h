#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct rv3028_t *rv3028_handle_t;

    typedef struct
    {
        i2c_master_bus_handle_t bus;

        uint32_t i2c_clock_speed_hz; /* 0 selects the 400 kHz default */
    } rv3028_config_t;

    /**
     * Initialize the RV-3028-C7 RTC driver.
     *
     * Adds the chip to an existing I2C master bus and verifies communication
     * with it. Backup power switchover to VBACKUP is enabled (level switching)
     * and trickle charging is kept disabled for the non-rechargeable coin
     * cell; the chip's EEPROM is only written when the stored configuration
     * differs.
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success or an error from the I2C driver.
     */
    esp_err_t rv3028_init(const rv3028_config_t *config, rv3028_handle_t *out_handle);

    /**
     * Set the RTC date and time.
     *
     * Uses tm_year, tm_mon, tm_mday, tm_hour, tm_min and tm_sec; tm_wday is
     * computed from the date. Years 2000-2099 are supported. Setting the time
     * marks the RTC time as valid again after a power loss.
     *
     * @param handle Driver handle.
     * @param time Time to set.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out of range fields.
     */
    esp_err_t rv3028_set_time(rv3028_handle_t handle, const struct tm *time);

    /**
     * Get the RTC date and time.
     *
     * Fills tm_year, tm_mon, tm_mday, tm_wday, tm_hour, tm_min and tm_sec.
     *
     * @param handle Driver handle.
     * @param out_time Receives the current time.
     * @return ESP_OK on success.
     */
    esp_err_t rv3028_get_time(rv3028_handle_t handle, struct tm *out_time);

    /**
     * Query whether the RTC time is valid.
     *
     * The time is invalid when the chip lost both main and backup power since
     * it was last set (power-on reset flag).
     *
     * @param handle Driver handle.
     * @param out_valid Receives the validity state.
     * @return ESP_OK on success.
     */
    esp_err_t rv3028_is_time_valid(rv3028_handle_t handle, bool *out_valid);

    /**
     * Arm the daily alarm and route it to the INT pin.
     *
     * The weekday and date are masked out, so the alarm matches on the hour and
     * minute alone and therefore fires once a day. The INT pin is open drain and
     * pulls low when the alarm hits; it releases again when the alarm flag is
     * cleared with rv3028_clear_alarm_flag(). Any pending flag is cleared here,
     * so arming never leaves a stale interrupt behind.
     *
     * @param handle Driver handle.
     * @param hour Alarm hour, 0-23.
     * @param minute Alarm minute, 0-59.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out of range values.
     */
    esp_err_t rv3028_set_alarm(rv3028_handle_t handle, int hour, int minute);

    /**
     * Disarm the alarm and stop driving the INT pin from it.
     *
     * @param handle Driver handle.
     * @return ESP_OK on success.
     */
    esp_err_t rv3028_disable_alarm(rv3028_handle_t handle);

    /**
     * Read the alarm flag.
     *
     * The flag latches when the alarm matches and stays set, holding INT low,
     * until it is cleared.
     *
     * @param handle Driver handle.
     * @param out_triggered Receives the flag state.
     * @return ESP_OK on success.
     */
    esp_err_t rv3028_get_alarm_flag(rv3028_handle_t handle, bool *out_triggered);

    /**
     * Clear the alarm flag, releasing the INT pin.
     *
     * @param handle Driver handle.
     * @return ESP_OK on success.
     */
    esp_err_t rv3028_clear_alarm_flag(rv3028_handle_t handle);

#ifdef __cplusplus
}
#endif
