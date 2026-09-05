#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct vsense_t *vsense_handle_t;

/* Unity gain correction, the starting point for vsense_set_scale_ppm() */
#define VSENSE_SCALE_UNITY_PPM 1000000

    typedef struct
    {
        adc_oneshot_unit_handle_t adc_unit; /* shared ADC1 oneshot unit */

        int in_gpio_num; /* GPIO the divider tap drives; must be an ADC1 channel */

        /*
         * Resistor divider between the monitored rail and ground. The tap sits
         * between the two, so the rail voltage is the tap voltage scaled by
         * (high_side_ohms + low_side_ohms) / low_side_ohms.
         */
        uint32_t high_side_ohms;
        uint32_t low_side_ohms;

        uint32_t sample_count;    /* samples averaged per reading, 0 selects 16 */
        uint32_t undervoltage_mv; /* rail voltage below this is out of range, 0 disables */
        uint32_t overvoltage_mv;  /* rail voltage above this is out of range, 0 disables */
        uint32_t hysteresis_mv;   /* range hysteresis, 0 selects 200 mV */
    } vsense_config_t;

    typedef struct
    {
        int      pin_millivolts; /* voltage at the divider tap */
        uint32_t millivolts;     /* voltage at the monitored rail */
        bool     clipped;        /* the ADC hit full scale, millivolts is a lower bound */
        bool     in_range;       /* inside the configured window, with hysteresis applied */
    } vsense_reading_t;

    /**
     * Initialize a resistor divider rail voltage monitor.
     *
     * Configures the divider tap as a channel on the caller's oneshot ADC1 unit
     * at 12 dB attenuation and sets up eFuse calibration when the chip provides
     * it. The divider ratio comes from the configured resistor values; trim the
     * remaining error against a meter with vsense_calibrate().
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when in_gpio_num is not an
     *         ADC1 channel or the divider values do not make sense, or an error
     *         from the ADC driver.
     */
    esp_err_t vsense_init(const vsense_config_t *config, vsense_handle_t *out_handle);

    /**
     * Take a reading.
     *
     * Averages the configured number of ADC samples, scales them up through the
     * divider ratio and updates the hysteresis state used for range checking.
     *
     * @param handle Driver handle.
     * @param out_reading Receives the reading.
     * @return ESP_OK on success or an error from the ADC driver.
     */
    esp_err_t vsense_read(vsense_handle_t handle, vsense_reading_t *out_reading);

    /**
     * Read only the rail voltage, in millivolts.
     *
     * @param handle Driver handle.
     * @param out_millivolts Receives the rail voltage.
     * @return ESP_OK on success.
     */
    esp_err_t vsense_read_millivolts(vsense_handle_t handle, uint32_t *out_millivolts);

    /**
     * Read whether the rail is inside the configured window.
     *
     * The state latches with the configured hysteresis, so a rail sitting on a
     * limit does not chatter. A monitor with both limits disabled is always in
     * range.
     *
     * @param handle Driver handle.
     * @param out_in_range Receives the range state.
     * @return ESP_OK on success.
     */
    esp_err_t vsense_is_in_range(vsense_handle_t handle, bool *out_in_range);

    /**
     * Get the highest rail voltage the divider can report, in millivolts.
     *
     * Above this the ADC saturates and readings clip; it is a property of the
     * divider ratio and the ADC full scale, not of the monitored rail.
     *
     * @param handle Driver handle.
     * @param out_millivolts Receives the ceiling.
     * @return ESP_OK on success.
     */
    esp_err_t vsense_get_max_millivolts(vsense_handle_t handle, uint32_t *out_millivolts);

    /**
     * Trim the gain against a rail voltage measured with a meter.
     *
     * Absorbs the divider resistor tolerances and any residual ADC gain error.
     * The correction is not persisted; read it back with vsense_get_scale_ppm()
     * to store it, and restore it after a reboot with vsense_set_scale_ppm().
     *
     * @param handle Driver handle.
     * @param actual_millivolts Rail voltage as measured externally.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when actual_millivolts is
     *         0, ESP_ERR_INVALID_STATE when the pin reads 0 mV, or an error
     *         from the ADC driver.
     */
    esp_err_t vsense_calibrate(vsense_handle_t handle, uint32_t actual_millivolts);

    /**
     * Get the gain correction in parts per million.
     *
     * VSENSE_SCALE_UNITY_PPM means the nominal divider ratio is used as is.
     */
    esp_err_t vsense_get_scale_ppm(vsense_handle_t handle, uint32_t *out_scale_ppm);

    /**
     * Set the gain correction in parts per million.
     *
     * @param handle Driver handle.
     * @param scale_ppm Correction, within half and double unity.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when out of range.
     */
    esp_err_t vsense_set_scale_ppm(vsense_handle_t handle, uint32_t scale_ppm);

    /**
     * Set the acceptable rail voltage window, all values in millivolts.
     *
     * @param handle Driver handle.
     * @param undervoltage_mv Lower limit, 0 disables the lower limit.
     * @param overvoltage_mv Upper limit, 0 disables the upper limit.
     * @param hysteresis_mv Hysteresis around both limits, may be 0.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when the limits cross or
     *         the hysteresis is wider than the window.
     */
    esp_err_t vsense_set_range(vsense_handle_t handle, uint32_t undervoltage_mv, uint32_t overvoltage_mv,
                               uint32_t hysteresis_mv);

    /**
     * Get the acceptable rail voltage window, all values in millivolts.
     *
     * Any output pointer may be NULL.
     */
    esp_err_t vsense_get_range(vsense_handle_t handle, uint32_t *out_undervoltage_mv, uint32_t *out_overvoltage_mv,
                               uint32_t *out_hysteresis_mv);

#ifdef __cplusplus
}
#endif
