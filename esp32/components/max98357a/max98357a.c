#include "max98357a.h"

#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "melodies.h"

#define MAX98357A_SAMPLE_RATE_HZ    44100
#define MAX98357A_CHUNK_FRAMES      256
#define MAX98357A_ATTACK_MS         5
#define MAX98357A_RELEASE_MS        30
#define MAX98357A_FULL_SCALE        30000
#define MAX98357A_TASK_STACK_SIZE   4096
#define MAX98357A_TASK_PRIORITY     5

typedef struct max98357a_t
{
    i2s_chan_handle_t tx_chan;
    int               sd_mode_gpio_num;

    TaskHandle_t      task;
    SemaphoreHandle_t done_sem;
    volatile bool     stop_requested;
    volatile bool     playing;

    const melody_t *melody;
    uint32_t        repeat_count;
    uint32_t        volume_pct;
} max98357a_ctx_t;

static const char *TAG = "max98357a";

static esp_err_t max98357a_init_i2s(const max98357a_config_t *config, i2s_chan_handle_t *out_chan);
static esp_err_t max98357a_stop_locked(max98357a_ctx_t *ctx);
static void      max98357a_shutdown(max98357a_ctx_t *ctx, bool enable);
static void      max98357a_play_task(void *arg);
static bool      max98357a_play_note(max98357a_ctx_t *ctx, const melody_note_t *note, int16_t *chunk);

esp_err_t max98357a_init(const max98357a_config_t *config, max98357a_handle_t *out_handle)
{
    esp_err_t        err;
    max98357a_ctx_t *ctx;

    if (!config || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(max98357a_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->sd_mode_gpio_num = config->sd_mode_gpio_num;

    ctx->done_sem = xSemaphoreCreateBinary();
    if (!ctx->done_sem)
    {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    if (ctx->sd_mode_gpio_num >= 0)
    {
        gpio_config_t sd_mode_config = {
            .pin_bit_mask = 1ULL << ctx->sd_mode_gpio_num,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };

        err = gpio_config(&sd_mode_config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure SD_MODE GPIO (%s)", esp_err_to_name(err));
            vSemaphoreDelete(ctx->done_sem);
            free(ctx);
            return err;
        }

        gpio_set_level(ctx->sd_mode_gpio_num, 0);
    }

    err = max98357a_init_i2s(config, &ctx->tx_chan);
    if (err != ESP_OK)
    {
        vSemaphoreDelete(ctx->done_sem);
        free(ctx);
        return err;
    }

    ESP_LOGI(TAG, "Initialized (BCLK on GPIO%d, LRCLK on GPIO%d, DIN on GPIO%d)", config->bclk_gpio_num,
             config->lrclk_gpio_num, config->din_gpio_num);

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t max98357a_play(max98357a_handle_t handle, max98357a_melody_t melody, uint32_t repeat_count,
                         uint32_t volume_pct)
{
    esp_err_t err;

    if (!handle || repeat_count == 0 || volume_pct > 100)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const melody_t *melody_data = melody_get(melody);
    if (!melody_data)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = max98357a_stop_locked(handle);
    if (err != ESP_OK)
    {
        return err;
    }

    handle->melody         = melody_data;
    handle->repeat_count   = repeat_count;
    handle->volume_pct     = volume_pct;
    handle->stop_requested = false;
    handle->playing        = true;

    if (xTaskCreate(max98357a_play_task, "max98357a", MAX98357A_TASK_STACK_SIZE, handle, MAX98357A_TASK_PRIORITY,
                    &handle->task) != pdPASS)
    {
        handle->playing = false;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t max98357a_stop(max98357a_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return max98357a_stop_locked(handle);
}

esp_err_t max98357a_is_playing(max98357a_handle_t handle, bool *out_playing)
{
    if (!handle || !out_playing)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_playing = handle->playing;
    return ESP_OK;
}

static esp_err_t max98357a_init_i2s(const max98357a_config_t *config, i2s_chan_handle_t *out_chan)
{
    esp_err_t err;

    i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(config->i2s_port, I2S_ROLE_MASTER);

    err = i2s_new_channel(&chan_config, out_chan, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2S channel (%s)", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_config = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MAX98357A_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->bclk_gpio_num,
            .ws   = config->lrclk_gpio_num,
            .dout = config->din_gpio_num,
            .din  = I2S_GPIO_UNUSED,
        },
    };

    err = i2s_channel_init_std_mode(*out_chan, &std_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode (%s)", esp_err_to_name(err));
        i2s_del_channel(*out_chan);
        *out_chan = NULL;
        return err;
    }

    return ESP_OK;
}

static esp_err_t max98357a_stop_locked(max98357a_ctx_t *ctx)
{
    if (!ctx->playing)
    {
        return ESP_OK;
    }

    ctx->stop_requested = true;

    if (xSemaphoreTake(ctx->done_sem, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Playback task did not stop in time");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void max98357a_shutdown(max98357a_ctx_t *ctx, bool enable)
{
    if (ctx->sd_mode_gpio_num >= 0)
    {
        gpio_set_level(ctx->sd_mode_gpio_num, enable ? 1 : 0);
    }
}

static void max98357a_play_task(void *arg)
{
    max98357a_ctx_t *ctx = arg;
    esp_err_t        err;
    int16_t         *chunk;

    chunk = malloc(MAX98357A_CHUNK_FRAMES * 2 * sizeof(int16_t));
    if (!chunk)
    {
        ESP_LOGE(TAG, "Failed to allocate sample buffer");
        goto out;
    }

    max98357a_shutdown(ctx, true);

    err = i2s_channel_enable(ctx->tx_chan);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable I2S channel (%s)", esp_err_to_name(err));
        goto out_shutdown;
    }

    for (uint32_t repeat = 0; repeat < ctx->repeat_count && !ctx->stop_requested; repeat++)
    {
        for (size_t i = 0; i < ctx->melody->note_count && !ctx->stop_requested; i++)
        {
            if (!max98357a_play_note(ctx, &ctx->melody->notes[i], chunk))
            {
                break;
            }
        }
    }

    i2s_channel_disable(ctx->tx_chan);

out_shutdown:
    max98357a_shutdown(ctx, false);

out:
    free(chunk);
    ctx->playing = false;
    xSemaphoreGive(ctx->done_sem);
    vTaskDelete(NULL);
}

static bool max98357a_play_note(max98357a_ctx_t *ctx, const melody_note_t *note, int16_t *chunk)
{
    esp_err_t err;
    size_t    written;
    uint32_t  total_samples   = ((uint32_t)note->duration_ms * MAX98357A_SAMPLE_RATE_HZ) / 1000;
    uint32_t  attack_samples  = (MAX98357A_ATTACK_MS * MAX98357A_SAMPLE_RATE_HZ) / 1000;
    uint32_t  release_samples = (MAX98357A_RELEASE_MS * MAX98357A_SAMPLE_RATE_HZ) / 1000;
    float     volume          = (float)ctx->volume_pct / 100.0f;
    float     amplitude       = MAX98357A_FULL_SCALE * volume * volume;
    float     phase           = 0.0f;
    float     phase_step      = 2.0f * (float)M_PI * note->frequency_hz / MAX98357A_SAMPLE_RATE_HZ;

    if (release_samples > total_samples / 2)
    {
        release_samples = total_samples / 2;
    }

    for (uint32_t sample = 0; sample < total_samples;)
    {
        uint32_t frames = total_samples - sample;
        if (frames > MAX98357A_CHUNK_FRAMES)
        {
            frames = MAX98357A_CHUNK_FRAMES;
        }

        for (uint32_t i = 0; i < frames; i++, sample++)
        {
            int16_t value = 0;

            if (note->frequency_hz != 0)
            {
                float envelope = 1.0f;

                if (sample < attack_samples)
                {
                    envelope = (float)sample / attack_samples;
                }

                uint32_t remaining = total_samples - sample;
                if (remaining < release_samples)
                {
                    envelope = (float)remaining / release_samples;
                }

                value = (int16_t)(amplitude * envelope * sinf(phase));

                phase += phase_step;
                if (phase > 2.0f * (float)M_PI)
                {
                    phase -= 2.0f * (float)M_PI;
                }
            }

            chunk[2 * i]     = value;
            chunk[2 * i + 1] = value;
        }

        err = i2s_channel_write(ctx->tx_chan, chunk, frames * 2 * sizeof(int16_t), &written, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "I2S write failed (%s)", esp_err_to_name(err));
            return false;
        }

        if (ctx->stop_requested)
        {
            return false;
        }
    }

    return true;
}
