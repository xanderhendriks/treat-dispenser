#include "gc9a01a.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gc9a01a_private.h"

#define GC9A01A_DEFAULT_CLOCK_SPEED_HZ    40000000
#define GC9A01A_DEFAULT_CHUNK_BYTES       4096
#define GC9A01A_MIN_CHUNK_BYTES           (GC9A01A_WIDTH * 2) /* a whole row, so the arithmetic never degenerates */
#define GC9A01A_BACKLIGHT_DEFAULT_FREQ_HZ 5000
#define GC9A01A_BACKLIGHT_RESOLUTION      LEDC_TIMER_10_BIT
#define GC9A01A_BACKLIGHT_MAX_DUTY        ((1U << GC9A01A_BACKLIGHT_RESOLUTION) - 1)

/* Longest parameter list in the initialization table below */
#define GC9A01A_INIT_MAX_PARAMS 12

/*
 * Reset timing taken from the vendor sample code for this module. Far more
 * generous than the controller needs, and there is no reason to shave a
 * fifth of a second off a once per boot sequence to find out how much.
 */
#define GC9A01A_RESET_SETUP_MS    5
#define GC9A01A_RESET_LOW_MS      20
#define GC9A01A_RESET_RECOVERY_MS 150

#define GC9A01A_COLMOD_RGB565 0x05

typedef struct
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[GC9A01A_INIT_MAX_PARAMS];
} gc9a01a_init_cmd_t;

/*
 * Panel initialization, transcribed from the ER-TFTM1.28-1 sample code that
 * ships with the module. Most of these registers are undocumented in the
 * GC9A01A datasheet: they are the panel maker's gate timing, gamma and power
 * settings for this particular glass, so they are copied verbatim rather than
 * derived. The two entries this driver owns are left out and applied
 * afterwards instead, so that a later change to either does not mean editing
 * the vendor's table:
 *
 *   - 36h MADCTL, which carries the rotation,
 *   - E8h, whose top nibble is the source polarity pattern.
 *
 * The vendor also enables the tearing effect output (35h); this module's eight
 * pin header does not bring that pin out, so it is dropped. Colour inversion
 * (21h) is not dropped: the IPS glass here needs it for red to come out red.
 */
static const gc9a01a_init_cmd_t s_init_sequence[] = {
    /* Inter register enable, which is what unlocks the vendor register space that follows */
    {0xEF, 0, {0}},
    {0xEB, 1, {0x14}},
    {0xFE, 0, {0}},
    {0xEF, 0, {0}},
    {0xEB, 1, {0x14}},

    {0x84, 1, {0x40}},
    {0x85, 1, {0xFF}},
    {0x86, 1, {0xFF}},
    {0x87, 1, {0xFF}},
    {0x88, 1, {0x0A}},
    {0x89, 1, {0x21}},
    {0x8A, 1, {0x00}},
    {0x8B, 1, {0x80}},
    {0x8C, 1, {0x01}},
    {0x8D, 1, {0x01}},
    {0x8E, 1, {0xFF}},
    {0x8F, 1, {0xFF}},

    {0xB6, 2, {0x00, 0x00}},
    {GC9A01A_CMD_COLMOD, 1, {GC9A01A_COLMOD_RGB565}},

    {0x90, 4, {0x08, 0x08, 0x08, 0x08}},
    {0xBD, 1, {0x06}},
    {0xBC, 1, {0x00}},
    {0xFF, 3, {0x60, 0x01, 0x04}},

    /* Power control */
    {0xC3, 1, {0x13}},
    {0xC4, 1, {0x13}},
    {0xC9, 1, {0x22}},
    {0xBE, 1, {0x11}},
    {0xE1, 2, {0x10, 0x0E}},
    {0xDF, 3, {0x21, 0x0C, 0x02}},

    /* Gamma */
    {0xF0, 6, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
    {0xF1, 6, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
    {0xF2, 6, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
    {0xF3, 6, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},

    {0xED, 2, {0x1B, 0x0B}},
    {0xAE, 1, {0x77}},
    {0xCD, 1, {0x63}},
    {0x70, 9, {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03}},

    /* Gate timing */
    {0x62, 12, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70}},
    {0x63, 12, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70}},
    {0x64, 7, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}},
    {0x66, 10, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}},
    {0x67, 10, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}},
    {0x74, 7, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}},
    {0x98, 2, {0x3E, 0x07}},

    {GC9A01A_CMD_INVON, 0, {0}},

    /*
     * Vertical scrolling over the whole panel with no fixed areas, which is
     * also the reset default. Set anyway so that the nudge the screen care
     * layer applies through 37h has a defined meaning without depending on
     * what the vendor table happened to leave behind.
     */
    {GC9A01A_CMD_VSCRDEF, 4, {0x00, 0x00, 0x00, GC9A01A_HEIGHT}},
};

/*
 * MADCTL for each rotation, again from the vendor sample code. Not the
 * combination of mirror bits one would guess: 0 degrees sets ML rather than
 * clearing everything, which is how the glass is wired to the controller on
 * this module. BGR is set throughout, so the driver takes RGB565 and the panel
 * reads it back the right way round.
 */
static const uint8_t s_madctl[] = {
    0x18, /* 0 degrees */
    0x28, /* 90 */
    0x48, /* 180 */
    0x88, /* 270 */
};

static const char *TAG = "gc9a01a";

static esp_err_t gc9a01a_init_gpio(const gc9a01a_config_t *config);
static esp_err_t gc9a01a_init_spi(const gc9a01a_config_t *config, gc9a01a_ctx_t *ctx);
static esp_err_t gc9a01a_init_backlight(const gc9a01a_config_t *config, gc9a01a_ctx_t *ctx);
static esp_err_t gc9a01a_reset(gc9a01a_ctx_t *ctx);
static esp_err_t gc9a01a_run_init_sequence(gc9a01a_ctx_t *ctx);
static esp_err_t gc9a01a_write_cmd(gc9a01a_ctx_t *ctx, uint8_t cmd);
static esp_err_t gc9a01a_write_params(gc9a01a_ctx_t *ctx, uint8_t cmd, const uint8_t *params, size_t len);
static esp_err_t gc9a01a_write_pixels(gc9a01a_ctx_t *ctx, uint8_t cmd, size_t bytes);
static esp_err_t gc9a01a_set_window(gc9a01a_ctx_t *ctx, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
static esp_err_t gc9a01a_stream(gc9a01a_ctx_t *ctx, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                uint16_t color, const uint16_t *pixels);
static bool      gc9a01a_rect_fits(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

esp_err_t gc9a01a_init(const gc9a01a_config_t *config, gc9a01a_handle_t *out_handle)
{
    esp_err_t      err;
    gc9a01a_ctx_t *ctx;
    size_t         chunk_bytes;

    if (!config || !out_handle || config->rotation > GC9A01A_ROTATION_270 ||
        config->dot_inversion > GC9A01A_DOT_INVERSION_8_DOT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(gc9a01a_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->dc_gpio_num        = config->dc_gpio_num;
    ctx->rst_gpio_num       = config->rst_gpio_num;
    ctx->backlight_gpio_num = config->backlight_gpio_num;
    ctx->rotation           = config->rotation;
    ctx->asleep             = false;

    chunk_bytes = config->transfer_chunk_bytes ? config->transfer_chunk_bytes : GC9A01A_DEFAULT_CHUNK_BYTES;
    if (chunk_bytes < GC9A01A_MIN_CHUNK_BYTES)
    {
        chunk_bytes = GC9A01A_MIN_CHUNK_BYTES;
    }
    chunk_bytes &= ~(size_t) 1; /* whole pixels only */

    ctx->dma_buf       = heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA);
    ctx->dma_buf_bytes = chunk_bytes;
    ctx->lock          = xSemaphoreCreateRecursiveMutex();

    if (!ctx->dma_buf || !ctx->lock)
    {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    err = gc9a01a_init_gpio(config);
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = gc9a01a_init_spi(config, ctx);
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = gc9a01a_init_backlight(config, ctx);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }

    err = gc9a01a_reset(ctx);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }

    err = gc9a01a_run_init_sequence(ctx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Panel initialization failed (%s)", esp_err_to_name(err));
        goto fail_spi;
    }

    err = gc9a01a_write_params(ctx, GC9A01A_CMD_MADCTL, &s_madctl[config->rotation], 1);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }

    err = gc9a01a_set_dot_inversion(ctx, config->dot_inversion);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }

    err = gc9a01a_write_cmd(ctx, GC9A01A_CMD_SLPOUT);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }
    vTaskDelay(pdMS_TO_TICKS(GC9A01A_SLEEP_SETTLE_MS));

    /*
     * Frame memory comes up holding whatever it holds, so it is cleared before
     * the display is switched on. The backlight stays off until the caller has
     * had a chance to draw something worth looking at.
     */
    err = gc9a01a_fill(ctx, GC9A01A_BLACK);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }

    err = gc9a01a_write_cmd(ctx, GC9A01A_CMD_DISPON);
    if (err != ESP_OK)
    {
        goto fail_spi;
    }

    ESP_LOGI(TAG, "Initialized (SCK on GPIO%d, MOSI on GPIO%d, CS on GPIO%d, DC on GPIO%d, RES on GPIO%d)",
             config->sck_gpio_num, config->mosi_gpio_num, config->cs_gpio_num, config->dc_gpio_num,
             config->rst_gpio_num);

    *out_handle = ctx;
    return ESP_OK;

fail_spi:
    spi_bus_remove_device(ctx->spi);
    spi_bus_free(config->spi_host);
fail:
    if (ctx->lock)
    {
        vSemaphoreDelete(ctx->lock);
    }
    free(ctx->dma_buf);
    free(ctx);
    return err;
}

esp_err_t gc9a01a_set_rotation(gc9a01a_handle_t handle, gc9a01a_rotation_t rotation)
{
    esp_err_t err;

    if (!handle || rotation > GC9A01A_ROTATION_270)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);
    err = gc9a01a_write_params(handle, GC9A01A_CMD_MADCTL, &s_madctl[rotation], 1);
    if (err == ESP_OK)
    {
        handle->rotation = rotation;
    }
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_get_rotation(gc9a01a_handle_t handle, gc9a01a_rotation_t *out_rotation)
{
    if (!handle || !out_rotation)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_rotation = handle->rotation;
    return ESP_OK;
}

esp_err_t gc9a01a_fill(gc9a01a_handle_t handle, uint16_t color)
{
    return gc9a01a_fill_rect(handle, 0, 0, GC9A01A_WIDTH, GC9A01A_HEIGHT, color);
}

esp_err_t gc9a01a_fill_rect(gc9a01a_handle_t handle, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                            uint16_t color)
{
    esp_err_t err;

    if (!handle || !gc9a01a_rect_fits(x, y, width, height))
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);
    err = gc9a01a_stream(handle, x, y, width, height, color, NULL);
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_draw_bitmap(gc9a01a_handle_t handle, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                              const uint16_t *pixels)
{
    esp_err_t err;

    if (!handle || !pixels || !gc9a01a_rect_fits(x, y, width, height))
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);
    err = gc9a01a_stream(handle, x, y, width, height, 0, pixels);
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_set_backlight(gc9a01a_handle_t handle, uint8_t percent)
{
    return gc9a01a_fade_backlight(handle, percent, 0);
}

esp_err_t gc9a01a_fade_backlight(gc9a01a_handle_t handle, uint8_t percent, uint32_t duration_ms)
{
    esp_err_t err;

    if (!handle || percent > 100)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->backlight_gpio_num < 0)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    gc9a01a_lock(handle);
    err = gc9a01a_apply_backlight(handle, percent, duration_ms);
    if (err == ESP_OK)
    {
        handle->backlight_pct = percent;
    }
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_get_backlight(gc9a01a_handle_t handle, uint8_t *out_percent)
{
    if (!handle || !out_percent)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_percent = handle->backlight_pct;
    return ESP_OK;
}

esp_err_t gc9a01a_set_display_on(gc9a01a_handle_t handle, bool on)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);
    err = gc9a01a_write_cmd(handle, on ? GC9A01A_CMD_DISPON : GC9A01A_CMD_DISPOFF);
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_sleep(gc9a01a_handle_t handle, bool sleep)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);

    if (handle->asleep == sleep)
    {
        gc9a01a_unlock(handle);
        return ESP_OK;
    }

    err = gc9a01a_write_cmd(handle, sleep ? GC9A01A_CMD_SLPIN : GC9A01A_CMD_SLPOUT);
    if (err == ESP_OK)
    {
        handle->asleep = sleep;

        /*
         * Both directions have to settle before the panel will take another
         * command: on the way in it is draining the panel and stopping the
         * converter, on the way out it is starting them again.
         */
        vTaskDelay(pdMS_TO_TICKS(GC9A01A_SLEEP_SETTLE_MS));
    }

    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_set_inversion(gc9a01a_handle_t handle, bool inverted)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gc9a01a_lock(handle);
    err = gc9a01a_write_cmd(handle, inverted ? GC9A01A_CMD_INVON : GC9A01A_CMD_INVOFF);
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_set_dot_inversion(gc9a01a_handle_t handle, gc9a01a_dot_inversion_t inversion)
{
    esp_err_t err;
    uint8_t   param;

    if (!handle || inversion > GC9A01A_DOT_INVERSION_8_DOT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * E8h holds DINV in the top nibble and the line period in the bottom one.
     * The low nibble is the value the vendor sample code uses for this glass,
     * so only the pattern is ever varied here.
     */
    param = (uint8_t) ((inversion << 4) | 0x04);

    gc9a01a_lock(handle);
    err = gc9a01a_write_params(handle, GC9A01A_CMD_FRAMERT, &param, 1);
    gc9a01a_unlock(handle);

    return err;
}

esp_err_t gc9a01a_set_scroll_offset(gc9a01a_handle_t handle, uint16_t lines)
{
    esp_err_t err;
    uint8_t   params[2];

    if (!handle || lines >= GC9A01A_HEIGHT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    params[0] = (uint8_t) (lines >> 8);
    params[1] = (uint8_t) (lines & 0xFF);

    gc9a01a_lock(handle);
    err = gc9a01a_write_params(handle, GC9A01A_CMD_VSCRSADD, params, sizeof(params));
    if (err == ESP_OK)
    {
        handle->scroll_offset_px = lines;
    }
    gc9a01a_unlock(handle);

    return err;
}

void gc9a01a_lock(gc9a01a_ctx_t *ctx)
{
    xSemaphoreTakeRecursive(ctx->lock, portMAX_DELAY);
}

void gc9a01a_unlock(gc9a01a_ctx_t *ctx)
{
    xSemaphoreGiveRecursive(ctx->lock);
}

esp_err_t gc9a01a_apply_backlight(gc9a01a_ctx_t *ctx, uint8_t percent, uint32_t duration_ms)
{
    esp_err_t err;
    uint32_t  duty = (percent * GC9A01A_BACKLIGHT_MAX_DUTY) / 100;

    if (ctx->backlight_gpio_num < 0)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (duration_ms == 0)
    {
        err = ledc_set_duty(ctx->backlight_pwm_mode, ctx->backlight_pwm_channel, duty);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set backlight duty (%s)", esp_err_to_name(err));
            return err;
        }

        return ledc_update_duty(ctx->backlight_pwm_mode, ctx->backlight_pwm_channel);
    }

    err = ledc_set_fade_with_time(ctx->backlight_pwm_mode, ctx->backlight_pwm_channel, duty, (int) duration_ms);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set up backlight fade (%s)", esp_err_to_name(err));
        return err;
    }

    return ledc_fade_start(ctx->backlight_pwm_mode, ctx->backlight_pwm_channel, LEDC_FADE_WAIT_DONE);
}

static bool gc9a01a_rect_fits(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    if (width == 0 || height == 0)
    {
        return false;
    }

    return x + width <= GC9A01A_WIDTH && y + height <= GC9A01A_HEIGHT;
}

static esp_err_t gc9a01a_init_gpio(const gc9a01a_config_t *config)
{
    esp_err_t     err;
    uint64_t      pin_mask = 1ULL << config->dc_gpio_num;
    gpio_config_t io_config;

    if (config->rst_gpio_num >= 0)
    {
        pin_mask |= 1ULL << config->rst_gpio_num;
    }

    io_config = (gpio_config_t){
        .pin_bit_mask = pin_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&io_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure DC and RES (%s)", esp_err_to_name(err));
    }

    return err;
}

static esp_err_t gc9a01a_init_spi(const gc9a01a_config_t *config, gc9a01a_ctx_t *ctx)
{
    esp_err_t err;

    /*
     * MOSI and SCK land on the ESP32-S3 IOMUX pins for SPI2, so the signals
     * skip the GPIO matrix and its 40 MHz ceiling. CS goes through the matrix,
     * which it can afford to: it only has to settle once per transaction.
     * Nothing is read back, so no MISO is claimed.
     */
    spi_bus_config_t bus_config = {
        .mosi_io_num     = config->mosi_gpio_num,
        .miso_io_num     = -1,
        .sclk_io_num     = config->sck_gpio_num,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = (int) ctx->dma_buf_bytes,
    };

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = config->spi_clock_speed_hz ? config->spi_clock_speed_hz : GC9A01A_DEFAULT_CLOCK_SPEED_HZ,
        .mode           = 0,
        .spics_io_num   = config->cs_gpio_num,
        .queue_size     = 1,
    };

    err = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize the SPI bus (%s)", esp_err_to_name(err));
        return err;
    }

    err = spi_bus_add_device(config->spi_host, &device_config, &ctx->spi);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add the panel to the SPI bus (%s)", esp_err_to_name(err));
        spi_bus_free(config->spi_host);
        return err;
    }

    return ESP_OK;
}

static esp_err_t gc9a01a_init_backlight(const gc9a01a_config_t *config, gc9a01a_ctx_t *ctx)
{
    esp_err_t err;
    uint32_t  frequency_hz =
        config->backlight_pwm_frequency_hz ? config->backlight_pwm_frequency_hz : GC9A01A_BACKLIGHT_DEFAULT_FREQ_HZ;

    if (config->backlight_gpio_num < 0)
    {
        ESP_LOGW(TAG, "No BLK pin, so the backlight cannot be dimmed or switched off");
        return ESP_OK;
    }

    ctx->backlight_pwm_mode    = LEDC_LOW_SPEED_MODE;
    ctx->backlight_pwm_channel = config->backlight_pwm_channel;

    ledc_timer_config_t timer_config = {
        .speed_mode      = ctx->backlight_pwm_mode,
        .duty_resolution = GC9A01A_BACKLIGHT_RESOLUTION,
        .timer_num       = config->backlight_pwm_timer,
        .freq_hz         = frequency_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    err = ledc_timer_config(&timer_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure the backlight LEDC timer (%s)", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num   = config->backlight_gpio_num,
        .speed_mode = ctx->backlight_pwm_mode,
        .channel    = config->backlight_pwm_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = config->backlight_pwm_timer,
        .duty       = 0,
        .hpoint     = 0,
    };

    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure the backlight LEDC channel (%s)", esp_err_to_name(err));
        return err;
    }

    /*
     * Hardware fading, so that a ramp costs no CPU. The service is global and
     * some other driver may already have installed it, which is not a problem.
     */
    err = ledc_fade_func_install(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install the LEDC fade service (%s)", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t gc9a01a_reset(gc9a01a_ctx_t *ctx)
{
    esp_err_t err;

    if (ctx->rst_gpio_num < 0)
    {
        err = gc9a01a_write_cmd(ctx, GC9A01A_CMD_SWRESET);
        if (err != ESP_OK)
        {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(GC9A01A_RESET_RECOVERY_MS));
        return ESP_OK;
    }

    gpio_set_level(ctx->rst_gpio_num, 1);
    vTaskDelay(pdMS_TO_TICKS(GC9A01A_RESET_SETUP_MS));
    gpio_set_level(ctx->rst_gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(GC9A01A_RESET_LOW_MS));
    gpio_set_level(ctx->rst_gpio_num, 1);
    vTaskDelay(pdMS_TO_TICKS(GC9A01A_RESET_RECOVERY_MS));

    return ESP_OK;
}

static esp_err_t gc9a01a_run_init_sequence(gc9a01a_ctx_t *ctx)
{
    for (size_t i = 0; i < sizeof(s_init_sequence) / sizeof(s_init_sequence[0]); i++)
    {
        esp_err_t err =
            gc9a01a_write_params(ctx, s_init_sequence[i].cmd, s_init_sequence[i].data, s_init_sequence[i].len);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t gc9a01a_write_cmd(gc9a01a_ctx_t *ctx, uint8_t cmd)
{
    return gc9a01a_write_params(ctx, cmd, NULL, 0);
}

/*
 * A command byte with DC low, then its parameters with DC high. Both go out
 * with the polling API: these are a handful of bytes each, where waiting on an
 * interrupt costs more than spinning for them does.
 *
 * The parameters are copied into the DMA buffer first because the callers pass
 * pointers into flash, and the SPI driver will not hand flash to the DMA
 * engine. There is always room: no command takes more parameters than a panel
 * row has bytes.
 */
static esp_err_t gc9a01a_write_params(gc9a01a_ctx_t *ctx, uint8_t cmd, const uint8_t *params, size_t len)
{
    esp_err_t         err;
    spi_transaction_t transaction = {
        .flags   = SPI_TRANS_USE_TXDATA,
        .length  = 8,
        .tx_data = {cmd},
    };

    gpio_set_level(ctx->dc_gpio_num, 0);

    err = spi_device_polling_transmit(ctx->spi, &transaction);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write command 0x%02X (%s)", cmd, esp_err_to_name(err));
        return err;
    }

    if (len == 0)
    {
        return ESP_OK;
    }

    memcpy(ctx->dma_buf, params, len);

    transaction = (spi_transaction_t){
        .length    = len * 8,
        .tx_buffer = ctx->dma_buf,
    };

    gpio_set_level(ctx->dc_gpio_num, 1);

    err = spi_device_polling_transmit(ctx->spi, &transaction);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write %u parameters for command 0x%02X (%s)", (unsigned) len, cmd,
                 esp_err_to_name(err));
    }

    return err;
}

/*
 * One chunk of pixel data out of the DMA buffer, introduced by memory write
 * (2Ch) for the first chunk of a rectangle and memory write continue (3Ch) for
 * the rest. Chip select drops between chunks, and 3Ch is the command that says
 * to carry on from the pixel after the last one rather than start again.
 *
 * These transfers are kilobytes rather than bytes, so they go out through the
 * interrupt driven call and give the CPU back while the DMA engine works.
 */
static esp_err_t gc9a01a_write_pixels(gc9a01a_ctx_t *ctx, uint8_t cmd, size_t bytes)
{
    esp_err_t         err;
    spi_transaction_t transaction = {
        .flags   = SPI_TRANS_USE_TXDATA,
        .length  = 8,
        .tx_data = {cmd},
    };

    gpio_set_level(ctx->dc_gpio_num, 0);

    err = spi_device_polling_transmit(ctx->spi, &transaction);
    if (err != ESP_OK)
    {
        return err;
    }

    transaction = (spi_transaction_t){
        .length    = bytes * 8,
        .tx_buffer = ctx->dma_buf,
    };

    gpio_set_level(ctx->dc_gpio_num, 1);

    return spi_device_transmit(ctx->spi, &transaction);
}

static esp_err_t gc9a01a_set_window(gc9a01a_ctx_t *ctx, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    esp_err_t err;
    uint16_t  x_end = x + width - 1;
    uint16_t  y_end = y + height - 1;
    uint8_t   params[4];

    params[0] = (uint8_t) (x >> 8);
    params[1] = (uint8_t) (x & 0xFF);
    params[2] = (uint8_t) (x_end >> 8);
    params[3] = (uint8_t) (x_end & 0xFF);

    err = gc9a01a_write_params(ctx, GC9A01A_CMD_CASET, params, sizeof(params));
    if (err != ESP_OK)
    {
        return err;
    }

    params[0] = (uint8_t) (y >> 8);
    params[1] = (uint8_t) (y & 0xFF);
    params[2] = (uint8_t) (y_end >> 8);
    params[3] = (uint8_t) (y_end & 0xFF);

    return gc9a01a_write_params(ctx, GC9A01A_CMD_RASET, params, sizeof(params));
}

/*
 * Push a rectangle out through the DMA buffer, either one repeated colour or a
 * caller supplied image. RGB565 goes over the wire high byte first, which is
 * the other way round from how it sits in memory on this processor, so the
 * staging copy doubles as the byte swap. That is also what lets an image be a
 * const array in flash.
 */
static esp_err_t gc9a01a_stream(gc9a01a_ctx_t *ctx, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                uint16_t color, const uint16_t *pixels)
{
    esp_err_t err;
    size_t    remaining = (size_t) width * height;
    size_t    chunk_px  = ctx->dma_buf_bytes / 2;
    uint16_t *staging   = (uint16_t *) ctx->dma_buf;
    uint8_t   cmd       = GC9A01A_CMD_RAMWR;

    err = gc9a01a_set_window(ctx, x, y, width, height);
    if (err != ESP_OK)
    {
        return err;
    }

    if (!pixels)
    {
        /* A solid fill re-sends the same buffer, so it only has to be built once */
        size_t   prefill = remaining < chunk_px ? remaining : chunk_px;
        uint16_t swapped = (uint16_t) ((color << 8) | (color >> 8));

        for (size_t i = 0; i < prefill; i++)
        {
            staging[i] = swapped;
        }
    }

    while (remaining)
    {
        size_t count = remaining < chunk_px ? remaining : chunk_px;

        if (pixels)
        {
            for (size_t i = 0; i < count; i++)
            {
                uint16_t value = *pixels++;
                staging[i]     = (uint16_t) ((value << 8) | (value >> 8));
            }
        }

        err = gc9a01a_write_pixels(ctx, cmd, count * 2);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write pixel data (%s)", esp_err_to_name(err));
            return err;
        }

        remaining -= count;
        cmd = GC9A01A_CMD_RAMWRC;
    }

    return ESP_OK;
}
