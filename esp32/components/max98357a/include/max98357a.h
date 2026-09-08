#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2s_common.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct max98357a_t *max98357a_handle_t;

    typedef enum
    {
        MAX98357A_MELODY_FOR_THE_LONGEST_TIME = 0,
        MAX98357A_MELODY_TADA                 = 1,
        MAX98357A_MELODY_COUNT,
    } max98357a_melody_t;

    typedef struct
    {
        int bclk_gpio_num;
        int lrclk_gpio_num;
        int din_gpio_num;
        int sd_mode_gpio_num; /* -1 when not connected */

        i2s_port_t i2s_port;
    } max98357a_config_t;

    /**
     * Initialize the MAX98357A I2S amplifier driver.
     *
     * Sets up an I2S TX channel (44.1 kHz, 16 bit, stereo) and the SD_MODE
     * GPIO. The amplifier is held in shutdown until playback starts.
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success or an error from the I2S/GPIO drivers.
     */
    esp_err_t max98357a_init(const max98357a_config_t *config, max98357a_handle_t *out_handle);

    /**
     * Play a melody a number of times at the given volume.
     *
     * Playback runs in a background task; this function returns immediately.
     * A melody that is already playing is stopped first.
     *
     * @param handle Driver handle.
     * @param melody Melody to play.
     * @param repeat_count Number of times to play the melody (1 or more).
     * @param volume_pct Volume in percent (0-100).
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out of range values.
     */
    esp_err_t max98357a_play(max98357a_handle_t handle, max98357a_melody_t melody, uint32_t repeat_count,
                             uint32_t volume_pct);

    /**
     * Stop playback and shut the amplifier down.
     *
     * Blocks until the playback task has finished. A no-op when nothing is
     * playing.
     *
     * @param handle Driver handle.
     * @return ESP_OK on success.
     */
    esp_err_t max98357a_stop(max98357a_handle_t handle);

    /**
     * Query whether a melody is currently playing.
     */
    esp_err_t max98357a_is_playing(max98357a_handle_t handle, bool *out_playing);

#ifdef __cplusplus
}
#endif
