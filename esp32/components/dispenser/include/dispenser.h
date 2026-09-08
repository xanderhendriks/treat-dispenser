#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drv5055.h"
#include "drv8871.h"
#include "esp_err.h"
#include "max98357a.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct dispenser_t *dispenser_handle_t;

    /* Sound played once a move finishes; a repeat_count of 0 stays silent */
    typedef struct
    {
        max98357a_melody_t melody;
        uint32_t           repeat_count;
        uint32_t           volume_pct;
    } dispenser_chime_t;

    typedef struct
    {
        drv8871_handle_t   motor_handle; /* initialized DRV8871 handle */
        drv5055_handle_t   hall_handle;  /* initialized DRV5055 handle */
        max98357a_handle_t audio_handle; /* initialized MAX98357A handle, NULL to stay silent */

        dispenser_chime_t home_chime;    /* played when dispenser_home() reaches the home magnet */
        dispenser_chime_t advance_chime; /* played when dispenser_advance() reaches the next magnet */

        drv8871_direction_t direction;        /* direction the drum turns while dispensing */
        uint32_t            travel_speed_pct; /* motor speed used for moves, 0 selects 60 % */
        uint32_t            poll_interval_ms; /* Hall sampling interval, 0 selects 10 ms */
        uint32_t            timeout_ms;       /* budget per magnet-to-magnet hop, 0 selects 5000 ms */
        uint32_t            brake_ms;         /* active braking once a magnet is found, 0 selects 150 ms */
    } dispenser_config_t;

    typedef struct
    {
        int32_t  field_ut;       /* field measured after the drum came to a stop */
        uint32_t magnets_passed; /* magnets skipped before the target one was reached */
        uint32_t elapsed_ms;     /* time the motor ran */
        bool     already_there;  /* the drum was already parked on the target magnet, it did not move */
    } dispenser_result_t;

    /**
     * Initialize the dispenser motion layer.
     *
     * Pairs the H-bridge with the Hall sensor so the drum can be moved from one
     * magnet to the next. The motor is not touched here; it keeps whatever state
     * drv8871_init() left it in.
     *
     * @param config Motion configuration.
     * @param out_handle Receives the handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle or a
     *         speed above 100, ESP_ERR_NO_MEM when the handle cannot be allocated.
     */
    esp_err_t dispenser_init(const dispenser_config_t *config, dispenser_handle_t *out_handle);

    /**
     * Turn the drum until it parks on the home magnet.
     *
     * The home magnet is the one mounted with the opposite polarity to all the
     * others, so it is the only one that reads as a negative field. Magnets of
     * the normal polarity are counted and driven past. Returns immediately when
     * the drum is already parked on the home magnet.
     *
     * Blocks until the magnet is found or the move times out, and always leaves
     * the motor stopped. On success the configured home chime starts playing in
     * the background, including when the drum was already there.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT when no magnet showed up within
     *         the configured budget, ESP_ERR_NOT_FOUND when a full drum of
     *         magnets went by without a negative one, or an error from the motor
     *         or Hall driver.
     */
    esp_err_t dispenser_home(dispenser_handle_t handle, dispenser_result_t *out_result);

    /**
     * Turn the drum on to the next magnet, whatever its polarity.
     *
     * Always moves: a drum that is already parked on a magnet first drives clear
     * of it, so this advances exactly one position per call.
     *
     * Blocks until the magnet is found or the move times out, and always leaves
     * the motor stopped. On success the configured advance chime starts playing
     * in the background.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT when the next magnet did not
     *         arrive within the configured budget, or an error from the motor or
     *         Hall driver.
     */
    esp_err_t dispenser_advance(dispenser_handle_t handle, dispenser_result_t *out_result);

    /**
     * Read whether the drum is currently parked on the home magnet.
     *
     * @param handle Dispenser handle.
     * @param out_home Receives the result.
     * @return ESP_OK on success or an error from the Hall driver.
     */
    esp_err_t dispenser_is_home(dispenser_handle_t handle, bool *out_home);

    /**
     * Set the motor speed used for moves, in percent.
     *
     * @param handle Dispenser handle.
     * @param speed_pct Speed in percent (1-100).
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out of range values.
     */
    esp_err_t dispenser_set_travel_speed(dispenser_handle_t handle, uint32_t speed_pct);

    /**
     * Get the motor speed used for moves, in percent.
     */
    esp_err_t dispenser_get_travel_speed(dispenser_handle_t handle, uint32_t *out_speed_pct);

#ifdef __cplusplus
}
#endif
