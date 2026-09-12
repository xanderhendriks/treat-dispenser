#pragma once

#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gc9a01a.h"

/* Commands this driver issues. The vendor initialization table uses plenty more that the datasheet does not name. */
#define GC9A01A_CMD_SWRESET  0x01
#define GC9A01A_CMD_SLPIN    0x10
#define GC9A01A_CMD_SLPOUT   0x11
#define GC9A01A_CMD_INVOFF   0x20
#define GC9A01A_CMD_INVON    0x21
#define GC9A01A_CMD_DISPOFF  0x28
#define GC9A01A_CMD_DISPON   0x29
#define GC9A01A_CMD_CASET    0x2A
#define GC9A01A_CMD_RASET    0x2B
#define GC9A01A_CMD_RAMWR    0x2C
#define GC9A01A_CMD_VSCRDEF  0x33
#define GC9A01A_CMD_MADCTL   0x36
#define GC9A01A_CMD_VSCRSADD 0x37
#define GC9A01A_CMD_COLMOD   0x3A
#define GC9A01A_CMD_RAMWRC   0x3C
#define GC9A01A_CMD_FRAMERT  0xE8

/* Sleep in and sleep out both need the supplies and the oscillator to settle, and the datasheet is specific about it */
#define GC9A01A_SLEEP_SETTLE_MS 120

typedef struct
{
    TaskHandle_t         task;
    gc9a01a_care_state_t state;

    uint32_t nudge_after_ms;
    uint32_t dim_after_ms;
    uint32_t blank_after_ms;
    uint32_t warn_after_ms;
    uint8_t  dim_backlight_pct;
    uint8_t  nudge_amplitude_px;

    int64_t touched_us;
    int64_t nudged_us;
    int64_t soak_until_us;

    uint16_t nudge_offset_px;
    int8_t   nudge_step;
    bool     warned;
} gc9a01a_care_ctx_t;

typedef struct gc9a01a_t
{
    spi_device_handle_t spi;

    int dc_gpio_num;
    int rst_gpio_num;
    int backlight_gpio_num;

    ledc_mode_t    backlight_pwm_mode;
    ledc_channel_t backlight_pwm_channel;
    uint8_t        backlight_pct; /* what the application asked for, which the screen care layer does not disturb */

    gc9a01a_rotation_t rotation;
    uint16_t           scroll_offset_px;
    bool               asleep;

    uint8_t *dma_buf;
    size_t   dma_buf_bytes;

    /*
     * Recursive, because the screen care task holds it across a whole
     * transition and reaches the panel through the same public calls the
     * application uses.
     */
    SemaphoreHandle_t lock;

    gc9a01a_care_ctx_t *care;
} gc9a01a_ctx_t;

void gc9a01a_lock(gc9a01a_ctx_t *ctx);
void gc9a01a_unlock(gc9a01a_ctx_t *ctx);

/**
 * Drive the backlight without touching the level the application asked for.
 *
 * How the screen care layer dims and blanks: gc9a01a_set_backlight() still
 * reports what the application wanted, so there is something to come back to.
 */
esp_err_t gc9a01a_apply_backlight(gc9a01a_ctx_t *ctx, uint8_t percent, uint32_t duration_ms);
