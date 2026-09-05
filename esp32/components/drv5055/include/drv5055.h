#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct drv5055_t *drv5055_handle_t;

/*
 * Sensitivity of each DRV5055 option in microvolts per millitesla (typical
 * values at 25 degC from the datasheet magnetic characteristics table). The
 * part is ratiometric, so the sensitivity depends on the supply rail.
 */
#define DRV5055_SENSITIVITY_A1_3V3_UV_MT 60000
#define DRV5055_SENSITIVITY_A2_3V3_UV_MT 30000
#define DRV5055_SENSITIVITY_A3_3V3_UV_MT 15000
#define DRV5055_SENSITIVITY_A4_3V3_UV_MT 7500
#define DRV5055_SENSITIVITY_A8_3V3_UV_MT 40000

#define DRV5055_SENSITIVITY_A1_5V_UV_MT 100000
#define DRV5055_SENSITIVITY_A2_5V_UV_MT 50000
#define DRV5055_SENSITIVITY_A3_5V_UV_MT 25000
#define DRV5055_SENSITIVITY_A4_5V_UV_MT 12500
#define DRV5055_SENSITIVITY_A8_5V_UV_MT 66600

    typedef struct
    {
        adc_oneshot_unit_handle_t adc_unit; /* shared ADC1 oneshot unit */

        int out_gpio_num; /* GPIO the OUT pin drives; must be an ADC1 channel */

        int32_t  sensitivity_uv_mt; /* 0 selects the DRV5055A3 at 3.3 V */
        uint32_t supply_mv;         /* sensor supply rail, 0 selects 3300 mV */
        uint32_t sample_count;      /* samples averaged per reading, 0 selects 16 */
        int32_t  threshold_ut;      /* field magnitude that counts as a magnet, 0 selects 5000 uT */
        int32_t  hysteresis_ut;     /* detection hysteresis, 0 selects a quarter of the threshold */
    } drv5055_config_t;

    typedef struct
    {
        int     millivolts;     /* voltage at the OUT pin */
        int32_t field_ut;       /* flux density, positive when a south pole faces the marked side */
        bool    saturated;      /* output left the linear window, field_ut is clipped */
        bool    magnet_present; /* detection threshold crossed, with hysteresis applied */
    } drv5055_reading_t;

    /**
     * Initialize the DRV5055 linear Hall sensor driver.
     *
     * Configures the OUT pin as a channel on the caller's oneshot ADC1 unit at
     * 12 dB attenuation, which covers the whole 0 V to VCC swing, and sets up
     * eFuse calibration when the chip provides it. The zero-field reference
     * starts at the nominal VCC/2; call drv5055_calibrate_zero() with no magnet
     * nearby to replace it with the real quiescent voltage.
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when out_gpio_num is not an
     *         ADC1 channel, or an error from the ADC driver.
     */
    esp_err_t drv5055_init(const drv5055_config_t *config, drv5055_handle_t *out_handle);

    /**
     * Take a reading.
     *
     * Averages the configured number of ADC samples, converts them to a flux
     * density and updates the hysteresis state used for magnet detection.
     *
     * @param handle Driver handle.
     * @param out_reading Receives the reading.
     * @return ESP_OK on success or an error from the ADC driver.
     */
    esp_err_t drv5055_read(drv5055_handle_t handle, drv5055_reading_t *out_reading);

    /**
     * Read only the flux density, in microtesla.
     *
     * Positive values mean a south pole faces the marked side of the package.
     *
     * @param handle Driver handle.
     * @param out_field_ut Receives the flux density.
     * @return ESP_OK on success.
     */
    esp_err_t drv5055_read_field_ut(drv5055_handle_t handle, int32_t *out_field_ut);

    /**
     * Read only the voltage at the OUT pin, in millivolts.
     *
     * @param handle Driver handle.
     * @param out_millivolts Receives the voltage.
     * @return ESP_OK on success.
     */
    esp_err_t drv5055_read_millivolts(drv5055_handle_t handle, int *out_millivolts);

    /**
     * Read whether a magnet is in front of the sensor.
     *
     * The state latches with the configured hysteresis, so a field hovering
     * around the threshold does not chatter.
     *
     * @param handle Driver handle.
     * @param out_present Receives the detection state.
     * @return ESP_OK on success.
     */
    esp_err_t drv5055_is_magnet_present(drv5055_handle_t handle, bool *out_present);

    /**
     * Capture the current output voltage as the zero-field reference.
     *
     * Removes the combined offset of the sensor quiescent voltage, the supply
     * rail being off nominal and the ADC. Must be called with no magnet near
     * the sensor. The captured value is not persisted; read it back with
     * drv5055_get_zero_millivolts() to store it, and restore it after a reboot
     * with drv5055_set_zero_millivolts().
     *
     * @param handle Driver handle.
     * @return ESP_OK on success or an error from the ADC driver.
     */
    esp_err_t drv5055_calibrate_zero(drv5055_handle_t handle);

    /**
     * Get the zero-field reference voltage in millivolts.
     */
    esp_err_t drv5055_get_zero_millivolts(drv5055_handle_t handle, int *out_millivolts);

    /**
     * Set the zero-field reference voltage in millivolts.
     *
     * @param handle Driver handle.
     * @param millivolts Reference voltage, must be within the supply rail.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when out of range.
     */
    esp_err_t drv5055_set_zero_millivolts(drv5055_handle_t handle, int millivolts);

    /**
     * Set the magnet detection threshold and hysteresis, both in microtesla.
     *
     * @param handle Driver handle.
     * @param threshold_ut Field magnitude that counts as a magnet.
     * @param hysteresis_ut Hysteresis around the threshold, may be 0.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for negative values or a
     *         hysteresis that is not smaller than the threshold.
     */
    esp_err_t drv5055_set_threshold(drv5055_handle_t handle, int32_t threshold_ut, int32_t hysteresis_ut);

    /**
     * Get the magnet detection threshold and hysteresis, both in microtesla.
     *
     * Either output pointer may be NULL.
     */
    esp_err_t drv5055_get_threshold(drv5055_handle_t handle, int32_t *out_threshold_ut, int32_t *out_hysteresis_ut);

#ifdef __cplusplus
}
#endif
