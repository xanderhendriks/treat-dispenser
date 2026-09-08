#include "dispenser.h"

#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DISPENSER_DEFAULT_SPEED_PCT  60
#define DISPENSER_DEFAULT_POLL_MS    10
#define DISPENSER_DEFAULT_TIMEOUT_MS 5000
#define DISPENSER_DEFAULT_BRAKE_MS   150

#define DISPENSER_MAX_SPEED_PCT 100

/*
 * Bound on how far dispenser_home() keeps hunting. A drum holds far fewer
 * magnets than this, so hitting the bound means the home magnet is missing or
 * mounted the wrong way round rather than that the search needs more time.
 */
#define DISPENSER_MAX_MAGNETS_PASSED 32

typedef struct dispenser_t
{
    drv8871_handle_t   motor;
    drv5055_handle_t   hall;
    max98357a_handle_t audio;

    dispenser_chime_t home_chime;
    dispenser_chime_t advance_chime;

    drv8871_direction_t direction;
    uint32_t            speed_pct;
    uint32_t            poll_interval_ms;
    uint32_t            timeout_ms;
    uint32_t            brake_ms;
} dispenser_ctx_t;

static const char *TAG = "dispenser";

static esp_err_t dispenser_seek(dispenser_ctx_t *ctx, bool home_only, dispenser_result_t *out_result);
static void      dispenser_play_chime(dispenser_ctx_t *ctx, const dispenser_chime_t *chime);
static esp_err_t dispenser_start_motor(dispenser_ctx_t *ctx);
static esp_err_t dispenser_stop_motor(dispenser_ctx_t *ctx, bool brake);
static bool      dispenser_is_home_field(int32_t field_ut);
static bool      dispenser_within_magnet(dispenser_ctx_t *ctx, int32_t field_ut);

esp_err_t dispenser_init(const dispenser_config_t *config, dispenser_handle_t *out_handle)
{
    dispenser_ctx_t *ctx;

    if (!config || !config->motor_handle || !config->hall_handle || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->travel_speed_pct > DISPENSER_MAX_SPEED_PCT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->direction != DRV8871_DIRECTION_FORWARD && config->direction != DRV8871_DIRECTION_REVERSE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(dispenser_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->motor            = config->motor_handle;
    ctx->hall             = config->hall_handle;
    ctx->audio            = config->audio_handle;
    ctx->home_chime       = config->home_chime;
    ctx->advance_chime    = config->advance_chime;
    ctx->direction        = config->direction;
    ctx->speed_pct        = config->travel_speed_pct ? config->travel_speed_pct : DISPENSER_DEFAULT_SPEED_PCT;
    ctx->poll_interval_ms = config->poll_interval_ms ? config->poll_interval_ms : DISPENSER_DEFAULT_POLL_MS;
    ctx->timeout_ms       = config->timeout_ms ? config->timeout_ms : DISPENSER_DEFAULT_TIMEOUT_MS;
    ctx->brake_ms         = config->brake_ms ? config->brake_ms : DISPENSER_DEFAULT_BRAKE_MS;

    ESP_LOGI(TAG, "Initialized (%s at %lu%%, %lu ms per magnet)",
             ctx->direction == DRV8871_DIRECTION_REVERSE ? "reverse" : "forward", (unsigned long) ctx->speed_pct,
             (unsigned long) ctx->timeout_ms);

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t dispenser_home(dispenser_handle_t handle, dispenser_result_t *out_result)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = dispenser_seek(handle, true, out_result);
    if (err == ESP_OK)
    {
        dispenser_play_chime(handle, &handle->home_chime);
    }

    return err;
}

esp_err_t dispenser_advance(dispenser_handle_t handle, dispenser_result_t *out_result)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = dispenser_seek(handle, false, out_result);
    if (err == ESP_OK)
    {
        dispenser_play_chime(handle, &handle->advance_chime);
    }

    return err;
}

esp_err_t dispenser_is_home(dispenser_handle_t handle, bool *out_home)
{
    esp_err_t         err;
    drv5055_reading_t reading;

    if (!handle || !out_home)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_read(handle->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_home = reading.magnet_present && dispenser_is_home_field(reading.field_ut);
    return ESP_OK;
}

esp_err_t dispenser_set_travel_speed(dispenser_handle_t handle, uint32_t speed_pct)
{
    if (!handle || speed_pct == 0 || speed_pct > DISPENSER_MAX_SPEED_PCT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->speed_pct = speed_pct;
    return ESP_OK;
}

esp_err_t dispenser_get_travel_speed(dispenser_handle_t handle, uint32_t *out_speed_pct)
{
    if (!handle || !out_speed_pct)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_speed_pct = handle->speed_pct;
    return ESP_OK;
}

/*
 * The chime is an acknowledgement, not part of the move: playback runs in its
 * own task and a failure to start it is logged rather than reported, so a mute
 * amplifier never turns a successful move into an error.
 */
static void dispenser_play_chime(dispenser_ctx_t *ctx, const dispenser_chime_t *chime)
{
    esp_err_t err;

    if (!ctx->audio || chime->repeat_count == 0)
    {
        return;
    }

    err = max98357a_play(ctx->audio, chime->melody, chime->repeat_count, chime->volume_pct);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to play the chime (%s)", esp_err_to_name(err));
    }
}

/*
 * The home magnet faces the sensor with the opposite pole to the others, so it
 * is the only one that pushes the DRV5055 output below its zero-field
 * reference. Every other magnet reads positive.
 */
static bool dispenser_is_home_field(int32_t field_ut)
{
    return field_ut < 0;
}

/*
 * Whether the drum is still close enough to a magnet for the next detection to
 * be that same magnet. The DRV5055 presence flag latches with hysteresis, so a
 * drum parked just under the trip point reads as no magnet even though it is
 * sitting on one; comparing against the lower hysteresis bound catches that and
 * keeps a move from stopping straight back where it started.
 */
static bool dispenser_within_magnet(dispenser_ctx_t *ctx, int32_t field_ut)
{
    int32_t threshold_ut  = 0;
    int32_t hysteresis_ut = 0;
    int32_t magnitude     = field_ut < 0 ? -field_ut : field_ut;

    if (drv5055_get_threshold(ctx->hall, &threshold_ut, &hysteresis_ut) != ESP_OK)
    {
        return false;
    }

    return magnitude > threshold_ut - hysteresis_ut;
}

/*
 * Both moves are the same hunt: drive off whatever magnet the drum is parked
 * on, then stop on the first magnet that satisfies the caller. Detection rides
 * on the hysteresis the DRV5055 driver already applies, so a magnet counts once
 * on the way in and only counts again after the field has fallen away.
 *
 * The timeout is a budget per hop rather than for the whole move, so homing
 * past several magnets does not have to outrun a single deadline.
 */
static esp_err_t dispenser_seek(dispenser_ctx_t *ctx, bool home_only, dispenser_result_t *out_result)
{
    esp_err_t          err;
    drv5055_reading_t  reading;
    dispenser_result_t result = {0};
    bool               clear_first;
    TickType_t         start;
    TickType_t         deadline;

    err = drv5055_read(ctx->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    if (home_only && reading.magnet_present && dispenser_is_home_field(reading.field_ut))
    {
        result.field_ut      = reading.field_ut;
        result.already_there = true;

        ESP_LOGI(TAG, "Already parked on the home magnet");

        if (out_result)
        {
            *out_result = result;
        }

        return ESP_OK;
    }

    clear_first = reading.magnet_present || dispenser_within_magnet(ctx, reading.field_ut);

    err = dispenser_start_motor(ctx);
    if (err != ESP_OK)
    {
        return err;
    }

    start    = xTaskGetTickCount();
    deadline = start + pdMS_TO_TICKS(ctx->timeout_ms);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ctx->poll_interval_ms));

        err = drv5055_read(ctx->hall, &reading);
        if (err != ESP_OK)
        {
            dispenser_stop_motor(ctx, false);
            return err;
        }

        if (clear_first)
        {
            /* Waiting for the field of the magnet the drum started on to fall away */
            if (!reading.magnet_present && !dispenser_within_magnet(ctx, reading.field_ut))
            {
                clear_first = false;
                deadline    = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);
            }
        }
        else if (reading.magnet_present)
        {
            if (!home_only || dispenser_is_home_field(reading.field_ut))
            {
                break;
            }

            /* Wrong polarity: count it and carry on to the next one */
            result.magnets_passed++;
            if (result.magnets_passed >= DISPENSER_MAX_MAGNETS_PASSED)
            {
                dispenser_stop_motor(ctx, true);
                ESP_LOGE(TAG, "No home magnet after %lu magnets", (unsigned long) result.magnets_passed);
                return ESP_ERR_NOT_FOUND;
            }

            clear_first = true;
            deadline    = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);
        }

        if (xTaskGetTickCount() >= deadline)
        {
            dispenser_stop_motor(ctx, true);
            ESP_LOGE(TAG, "No magnet within %lu ms, the drum may be jammed", (unsigned long) ctx->timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }

    result.elapsed_ms = (uint32_t) ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);

    err = dispenser_stop_motor(ctx, true);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Report the field where the drum came to rest, not where it tripped the threshold */
    err = drv5055_read(ctx->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    result.field_ut = reading.field_ut;

    ESP_LOGI(TAG, "Stopped on a magnet at %ld uT after %lu ms (%lu passed)", (long) result.field_ut,
             (unsigned long) result.elapsed_ms, (unsigned long) result.magnets_passed);

    if (out_result)
    {
        *out_result = result;
    }

    return ESP_OK;
}

static esp_err_t dispenser_start_motor(dispenser_ctx_t *ctx)
{
    esp_err_t err = drv8871_set_direction(ctx->motor, ctx->direction);
    if (err != ESP_OK)
    {
        return err;
    }

    return drv8871_set_speed(ctx->motor, ctx->speed_pct);
}

/*
 * Braking pulls the drum up short so it parks close to where the magnet was
 * detected; the brake is released afterwards so the H-bridge is not left
 * holding the winding shorted.
 */
static esp_err_t dispenser_stop_motor(dispenser_ctx_t *ctx, bool brake)
{
    esp_err_t err;

    if (!brake)
    {
        return drv8871_coast(ctx->motor);
    }

    err = drv8871_brake(ctx->motor);
    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(ctx->brake_ms));

    return drv8871_coast(ctx->motor);
}
