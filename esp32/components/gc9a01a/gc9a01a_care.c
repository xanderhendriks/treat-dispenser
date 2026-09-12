#include <inttypes.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gc9a01a.h"
#include "gc9a01a_private.h"

#define GC9A01A_CARE_DEFAULT_STACK_SIZE 3072
#define GC9A01A_CARE_DEFAULT_PRIORITY   3

/*
 * How often the policy is re-evaluated. Everything it does is measured in
 * minutes, so once a second is already far finer than it needs to be; it keeps
 * the reported static time honest for anyone reading the status.
 */
#define GC9A01A_CARE_TICK_MS 1000

/*
 * Defaults, all of them straight out of the ER-TFTM1.28-1 datasheet: shift a
 * long lived border now and then, screensaver in at five to ten idle minutes,
 * and never leave a fixed image up for two hours.
 */
#define GC9A01A_CARE_DEFAULT_NUDGE_AFTER_MS 60000
#define GC9A01A_CARE_DEFAULT_DIM_AFTER_MS   300000
#define GC9A01A_CARE_DEFAULT_BLANK_AFTER_MS 600000
#define GC9A01A_CARE_DEFAULT_WARN_AFTER_MS  7200000
#define GC9A01A_CARE_DEFAULT_DIM_PCT        20
#define GC9A01A_CARE_DEFAULT_NUDGE_PX       2

/* Past a handful of pixels a shift stops reading as a nudge and starts reading as a glitch */
#define GC9A01A_CARE_MAX_NUDGE_PX 16

#define GC9A01A_CARE_DIM_FADE_MS   1000
#define GC9A01A_CARE_BLANK_FADE_MS 600
#define GC9A01A_CARE_WAKE_FADE_MS  250

static const char *TAG = "gc9a01a";

static void     gc9a01a_care_task(void *arg);
static void     gc9a01a_care_tick(gc9a01a_ctx_t *ctx);
static void     gc9a01a_care_nudge(gc9a01a_ctx_t *ctx);
static void     gc9a01a_care_dim(gc9a01a_ctx_t *ctx);
static void     gc9a01a_care_blank(gc9a01a_ctx_t *ctx);
static void     gc9a01a_care_revive(gc9a01a_ctx_t *ctx);
static uint32_t gc9a01a_care_static_ms(const gc9a01a_care_ctx_t *care, int64_t now_us);

esp_err_t gc9a01a_care_start(gc9a01a_handle_t handle, const gc9a01a_care_config_t *config)
{
    gc9a01a_care_ctx_t   *care;
    gc9a01a_care_config_t defaults = {0};

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!config)
    {
        config = &defaults;
    }
    if (config->dim_backlight_pct > 100 || config->nudge_amplitude_px > GC9A01A_CARE_MAX_NUDGE_PX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);

    if (handle->care)
    {
        gc9a01a_unlock(handle);
        return ESP_ERR_INVALID_STATE;
    }

    care = calloc(1, sizeof(gc9a01a_care_ctx_t));
    if (!care)
    {
        gc9a01a_unlock(handle);
        return ESP_ERR_NO_MEM;
    }

    care->nudge_after_ms     = config->nudge_after_ms ? config->nudge_after_ms : GC9A01A_CARE_DEFAULT_NUDGE_AFTER_MS;
    care->dim_after_ms       = config->dim_after_ms ? config->dim_after_ms : GC9A01A_CARE_DEFAULT_DIM_AFTER_MS;
    care->blank_after_ms     = config->blank_after_ms ? config->blank_after_ms : GC9A01A_CARE_DEFAULT_BLANK_AFTER_MS;
    care->warn_after_ms      = config->warn_after_ms ? config->warn_after_ms : GC9A01A_CARE_DEFAULT_WARN_AFTER_MS;
    care->dim_backlight_pct  = config->dim_backlight_pct ? config->dim_backlight_pct : GC9A01A_CARE_DEFAULT_DIM_PCT;
    care->nudge_amplitude_px = config->nudge_amplitude_px ? config->nudge_amplitude_px : GC9A01A_CARE_DEFAULT_NUDGE_PX;

    care->state      = GC9A01A_CARE_STATE_ACTIVE;
    care->touched_us = esp_timer_get_time();
    care->nudged_us  = care->touched_us;

    handle->care = care;

    if (xTaskCreate(gc9a01a_care_task, "gc9a01a_care",
                    config->task_stack_size ? config->task_stack_size : GC9A01A_CARE_DEFAULT_STACK_SIZE, handle,
                    config->task_priority ? (UBaseType_t) config->task_priority : GC9A01A_CARE_DEFAULT_PRIORITY,
                    &care->task) != pdPASS)
    {
        handle->care = NULL;
        free(care);
        gc9a01a_unlock(handle);
        return ESP_ERR_NO_MEM;
    }

    gc9a01a_unlock(handle);

    ESP_LOGI(TAG, "Screen care running (nudge %" PRIu32 " s, dim %" PRIu32 " s, blank %" PRIu32 " s)",
             care->nudge_after_ms / 1000, care->dim_after_ms / 1000, care->blank_after_ms / 1000);

    return ESP_OK;
}

esp_err_t gc9a01a_care_touch(gc9a01a_handle_t handle)
{
    gc9a01a_care_ctx_t *care;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);

    care = handle->care;
    if (!care)
    {
        gc9a01a_unlock(handle);
        return ESP_OK;
    }

    care->touched_us = esp_timer_get_time();
    care->nudged_us  = care->touched_us;
    care->warned     = false;

    /*
     * A soak is a deliberate hours long thing, so an update arriving in the
     * middle of one does not cut it short. The drawing lands in frame memory
     * and shows up when the soak ends.
     */
    if (care->state != GC9A01A_CARE_STATE_SOAKING)
    {
        gc9a01a_care_revive(handle);
    }

    gc9a01a_unlock(handle);

    return ESP_OK;
}

esp_err_t gc9a01a_care_soak(gc9a01a_handle_t handle, uint32_t minutes)
{
    gc9a01a_care_ctx_t *care;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);

    care = handle->care;
    if (!care)
    {
        gc9a01a_unlock(handle);
        return ESP_ERR_INVALID_STATE;
    }

    if (minutes == 0)
    {
        if (care->state == GC9A01A_CARE_STATE_SOAKING)
        {
            ESP_LOGI(TAG, "Soak stopped");
            care->soak_until_us = 0;
            gc9a01a_care_revive(handle);
        }

        gc9a01a_unlock(handle);
        return ESP_OK;
    }

    /*
     * Black on a lit panel does nothing for the crystal that black on a dark
     * one does not, so the backlight goes first. What matters is that the panel
     * stays awake and keeps driving that black, which is the datasheet's own
     * recipe for reversing a ghost, rather than sleeping and simply letting the
     * charge off.
     */
    gc9a01a_apply_backlight(handle, 0, GC9A01A_CARE_BLANK_FADE_MS);
    gc9a01a_sleep(handle, false);
    gc9a01a_set_scroll_offset(handle, 0);
    gc9a01a_fill(handle, GC9A01A_BLACK);
    gc9a01a_set_display_on(handle, true);

    care->nudge_offset_px = 0;
    care->nudge_step      = 0;
    care->state           = GC9A01A_CARE_STATE_SOAKING;
    care->soak_until_us   = esp_timer_get_time() + (int64_t) minutes * 60 * 1000 * 1000;

    gc9a01a_unlock(handle);

    ESP_LOGI(TAG, "Soaking the panel in black for %" PRIu32 " minutes to unwind image sticking", minutes);

    return ESP_OK;
}

esp_err_t gc9a01a_care_get_status(gc9a01a_handle_t handle, gc9a01a_care_status_t *out_status)
{
    gc9a01a_care_ctx_t *care;
    int64_t             now_us;

    if (!handle || !out_status)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);

    care = handle->care;
    if (!care)
    {
        gc9a01a_unlock(handle);
        return ESP_ERR_INVALID_STATE;
    }

    now_us = esp_timer_get_time();

    out_status->state           = care->state;
    out_status->static_ms       = gc9a01a_care_static_ms(care, now_us);
    out_status->nudge_offset_px = care->nudge_offset_px;
    out_status->soak_remaining_s =
        care->soak_until_us > now_us ? (uint32_t) ((care->soak_until_us - now_us) / 1000000) : 0;

    gc9a01a_unlock(handle);

    return ESP_OK;
}

static void gc9a01a_care_task(void *arg)
{
    gc9a01a_ctx_t *ctx = arg;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(GC9A01A_CARE_TICK_MS));
        gc9a01a_care_tick(ctx);
    }
}

/*
 * The lock is held across a whole transition rather than just around the state
 * fields. A blank is a backlight ramp, a scroll reset and a sleep command, and
 * an update arriving halfway through that would otherwise draw into a panel
 * that is about to be put to sleep without anything noticing.
 */
static void gc9a01a_care_tick(gc9a01a_ctx_t *ctx)
{
    gc9a01a_care_ctx_t *care;
    int64_t             now_us;
    uint32_t            static_ms;

    gc9a01a_lock(ctx);

    care   = ctx->care;
    now_us = esp_timer_get_time();

    if (care->state == GC9A01A_CARE_STATE_SOAKING)
    {
        if (now_us >= care->soak_until_us)
        {
            ESP_LOGI(TAG, "Soak finished, the panel is showing black until the next update paints over it");
            care->touched_us = now_us;
            care->nudged_us  = now_us;
            gc9a01a_care_revive(ctx);
        }

        gc9a01a_unlock(ctx);
        return;
    }

    static_ms = gc9a01a_care_static_ms(care, now_us);

    if (static_ms >= care->blank_after_ms)
    {
        if (care->state != GC9A01A_CARE_STATE_BLANKED)
        {
            gc9a01a_care_blank(ctx);
        }

        gc9a01a_unlock(ctx);
        return;
    }

    if (static_ms >= care->dim_after_ms && care->state != GC9A01A_CARE_STATE_DIMMED)
    {
        gc9a01a_care_dim(ctx);
    }

    /* The nudge keeps running once dimmed: a dim picture polarizes the crystal just as well as a bright one */
    if (care->nudge_after_ms != GC9A01A_CARE_NEVER && static_ms >= care->nudge_after_ms &&
        (uint32_t) ((now_us - care->nudged_us) / 1000) >= care->nudge_after_ms)
    {
        gc9a01a_care_nudge(ctx);
    }

    if (!care->warned && care->warn_after_ms != GC9A01A_CARE_NEVER && static_ms >= care->warn_after_ms)
    {
        ESP_LOGW(TAG,
                 "The same picture has been on the panel for %" PRIu32
                 " minutes; the panel datasheet expects it changed, dimmed or blanked well before this",
                 static_ms / 60000);
        care->warned = true;
    }

    gc9a01a_unlock(ctx);
}

/*
 * Walk the whole picture up and back down again by a couple of pixels, which
 * costs one register write and no repaint. The point is that the boundary
 * between two colours does not stand over the same row of liquid crystal for
 * hours, which is where the datasheet says sticking shows up worst.
 */
static void gc9a01a_care_nudge(gc9a01a_ctx_t *ctx)
{
    gc9a01a_care_ctx_t *care   = ctx->care;
    int8_t              period = (int8_t) (care->nudge_amplitude_px * 2);
    uint16_t            offset;

    care->nudge_step = (int8_t) ((care->nudge_step + 1) % period);
    offset = (uint16_t) (care->nudge_step <= care->nudge_amplitude_px ? care->nudge_step : period - care->nudge_step);

    if (gc9a01a_set_scroll_offset(ctx, offset) != ESP_OK)
    {
        return;
    }

    care->nudge_offset_px = offset;
    care->nudged_us       = esp_timer_get_time();

    if (care->state == GC9A01A_CARE_STATE_ACTIVE)
    {
        care->state = GC9A01A_CARE_STATE_NUDGED;
    }
}

static void gc9a01a_care_dim(gc9a01a_ctx_t *ctx)
{
    gc9a01a_care_ctx_t *care = ctx->care;

    if (ctx->backlight_gpio_num < 0)
    {
        return;
    }

    /* Only ever downwards: dimming to a level above what the application asked for is not dimming */
    if (care->dim_backlight_pct >= ctx->backlight_pct)
    {
        care->state = GC9A01A_CARE_STATE_DIMMED;
        return;
    }

    if (gc9a01a_apply_backlight(ctx, care->dim_backlight_pct, GC9A01A_CARE_DIM_FADE_MS) == ESP_OK)
    {
        care->state = GC9A01A_CARE_STATE_DIMMED;
    }
}

/*
 * Sleep mode rather than a black fill: it blanks the panel, stops the
 * converter and the oscillator, and drains the charge off the glass, which is
 * the one thing the controller can actively do about polarization. Frame memory
 * survives it, so the picture is still there and still right when the panel
 * comes back and nothing has to be redrawn to relight it.
 */
static void gc9a01a_care_blank(gc9a01a_ctx_t *ctx)
{
    gc9a01a_care_ctx_t *care = ctx->care;

    if (ctx->backlight_gpio_num >= 0)
    {
        gc9a01a_apply_backlight(ctx, 0, GC9A01A_CARE_BLANK_FADE_MS);
    }

    gc9a01a_set_scroll_offset(ctx, 0);
    care->nudge_offset_px = 0;
    care->nudge_step      = 0;

    if (gc9a01a_sleep(ctx, true) != ESP_OK)
    {
        return;
    }

    care->state = GC9A01A_CARE_STATE_BLANKED;

    ESP_LOGD(TAG, "Panel asleep after %" PRIu32 " s of a static picture", care->blank_after_ms / 1000);
}

/* Undo whatever was applied and put the panel back the way the application left it */
static void gc9a01a_care_revive(gc9a01a_ctx_t *ctx)
{
    gc9a01a_care_ctx_t *care  = ctx->care;
    bool                woken = false;

    if (care->nudge_offset_px)
    {
        gc9a01a_set_scroll_offset(ctx, 0);
        care->nudge_offset_px = 0;
        care->nudge_step      = 0;
    }

    if (ctx->asleep)
    {
        if (gc9a01a_sleep(ctx, false) != ESP_OK)
        {
            return;
        }

        woken = true;
    }

    if (ctx->backlight_gpio_num >= 0 && care->state != GC9A01A_CARE_STATE_ACTIVE)
    {
        gc9a01a_apply_backlight(ctx, ctx->backlight_pct, woken ? GC9A01A_CARE_WAKE_FADE_MS : 0);
    }

    care->state         = GC9A01A_CARE_STATE_ACTIVE;
    care->soak_until_us = 0;
}

static uint32_t gc9a01a_care_static_ms(const gc9a01a_care_ctx_t *care, int64_t now_us)
{
    return (uint32_t) ((now_us - care->touched_us) / 1000);
}
