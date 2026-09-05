#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct drv8871_t *drv8871_handle_t;

    typedef enum
    {
        DRV8871_DIRECTION_FORWARD = 0,
        DRV8871_DIRECTION_REVERSE = 1,
    } drv8871_direction_t;

    typedef struct
    {
        int in1_gpio_num;
        int in2_gpio_num;

        ledc_timer_t   pwm_timer;
        ledc_channel_t in1_pwm_channel;
        ledc_channel_t in2_pwm_channel;
        uint32_t       pwm_frequency_hz; /* 0 selects the 25 kHz default */
    } drv8871_config_t;

    /**
     * Initialize the DRV8871 H-bridge driver.
     *
     * Sets up one LEDC timer and two LEDC channels (one per input pin). The
     * motor is left coasting (both outputs low) after initialization.
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success or an error from the LEDC driver.
     */
    esp_err_t drv8871_init(const drv8871_config_t *config, drv8871_handle_t *out_handle);

    /**
     * Set the motor speed as a percentage of full duty cycle.
     *
     * A speed of 0 lets the motor coast. The configured direction determines
     * which input pin is driven with PWM.
     *
     * @param handle Driver handle.
     * @param speed_pct Speed in percent (0-100).
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out of range values.
     */
    esp_err_t drv8871_set_speed(drv8871_handle_t handle, uint32_t speed_pct);

    /**
     * Set the motor rotation direction.
     *
     * Takes effect immediately, also while the motor is running.
     *
     * @param handle Driver handle.
     * @param direction Rotation direction.
     * @return ESP_OK on success.
     */
    esp_err_t drv8871_set_direction(drv8871_handle_t handle, drv8871_direction_t direction);

    /**
     * Get the currently set speed in percent.
     */
    esp_err_t drv8871_get_speed(drv8871_handle_t handle, uint32_t *out_speed_pct);

    /**
     * Get the currently set direction.
     */
    esp_err_t drv8871_get_direction(drv8871_handle_t handle, drv8871_direction_t *out_direction);

    /**
     * Stop the motor by letting it coast (both outputs low), keeping the
     * configured direction for the next drv8871_set_speed() call.
     *
     * @param handle Driver handle.
     * @return ESP_OK on success.
     */
    esp_err_t drv8871_coast(drv8871_handle_t handle);

    /**
     * Actively brake the motor (both outputs high).
     *
     * @param handle Driver handle.
     * @return ESP_OK on success.
     */
    esp_err_t drv8871_brake(drv8871_handle_t handle);

#ifdef __cplusplus
}
#endif
