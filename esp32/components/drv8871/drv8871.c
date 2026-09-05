#include "drv8871.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_log.h"

#define DRV8871_PWM_RESOLUTION         LEDC_TIMER_10_BIT
#define DRV8871_PWM_MAX_DUTY           (1U << DRV8871_PWM_RESOLUTION)
#define DRV8871_PWM_DEFAULT_FREQ_HZ    25000

typedef struct drv8871_t
{
    ledc_mode_t         pwm_mode;
    ledc_channel_t      in1_pwm_channel;
    ledc_channel_t      in2_pwm_channel;
    uint32_t            speed_pct;
    drv8871_direction_t direction;
    bool                braking;
} drv8871_ctx_t;

static const char *TAG = "drv8871";

static esp_err_t drv8871_init_pwm(const drv8871_config_t *config, ledc_mode_t mode);
static esp_err_t drv8871_apply(drv8871_ctx_t *ctx);
static esp_err_t drv8871_set_duty(drv8871_ctx_t *ctx, ledc_channel_t channel, uint32_t duty);

esp_err_t drv8871_init(const drv8871_config_t *config, drv8871_handle_t *out_handle)
{
    esp_err_t      err;
    drv8871_ctx_t *ctx;

    if (!config || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(drv8871_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->pwm_mode        = LEDC_LOW_SPEED_MODE;
    ctx->in1_pwm_channel = config->in1_pwm_channel;
    ctx->in2_pwm_channel = config->in2_pwm_channel;
    ctx->speed_pct       = 0;
    ctx->direction       = DRV8871_DIRECTION_FORWARD;
    ctx->braking         = false;

    err = drv8871_init_pwm(config, ctx->pwm_mode);
    if (err != ESP_OK)
    {
        free(ctx);
        return err;
    }

    err = drv8871_apply(ctx);
    if (err != ESP_OK)
    {
        free(ctx);
        return err;
    }

    ESP_LOGI(TAG, "Initialized (IN1 on GPIO%d, IN2 on GPIO%d)", config->in1_gpio_num, config->in2_gpio_num);

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t drv8871_set_speed(drv8871_handle_t handle, uint32_t speed_pct)
{
    if (!handle || speed_pct > 100)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->speed_pct = speed_pct;
    handle->braking   = false;

    return drv8871_apply(handle);
}

esp_err_t drv8871_set_direction(drv8871_handle_t handle, drv8871_direction_t direction)
{
    if (!handle || (direction != DRV8871_DIRECTION_FORWARD && direction != DRV8871_DIRECTION_REVERSE))
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->direction = direction;

    return drv8871_apply(handle);
}

esp_err_t drv8871_get_speed(drv8871_handle_t handle, uint32_t *out_speed_pct)
{
    if (!handle || !out_speed_pct)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_speed_pct = handle->speed_pct;
    return ESP_OK;
}

esp_err_t drv8871_get_direction(drv8871_handle_t handle, drv8871_direction_t *out_direction)
{
    if (!handle || !out_direction)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_direction = handle->direction;
    return ESP_OK;
}

esp_err_t drv8871_coast(drv8871_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->speed_pct = 0;
    handle->braking   = false;

    return drv8871_apply(handle);
}

esp_err_t drv8871_brake(drv8871_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->speed_pct = 0;
    handle->braking   = true;

    return drv8871_apply(handle);
}

static esp_err_t drv8871_init_pwm(const drv8871_config_t *config, ledc_mode_t mode)
{
    esp_err_t err;
    uint32_t  frequency_hz = config->pwm_frequency_hz ? config->pwm_frequency_hz : DRV8871_PWM_DEFAULT_FREQ_HZ;

    ledc_timer_config_t timer_config = {
        .speed_mode      = mode,
        .duty_resolution = DRV8871_PWM_RESOLUTION,
        .timer_num       = config->pwm_timer,
        .freq_hz         = frequency_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    err = ledc_timer_config(&timer_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer (%s)", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t in1_channel_config = {
        .gpio_num   = config->in1_gpio_num,
        .speed_mode = mode,
        .channel    = config->in1_pwm_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = config->pwm_timer,
        .duty       = 0,
        .hpoint     = 0,
    };

    err = ledc_channel_config(&in1_channel_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure IN1 LEDC channel (%s)", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t in2_channel_config = {
        .gpio_num   = config->in2_gpio_num,
        .speed_mode = mode,
        .channel    = config->in2_pwm_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = config->pwm_timer,
        .duty       = 0,
        .hpoint     = 0,
    };

    err = ledc_channel_config(&in2_channel_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure IN2 LEDC channel (%s)", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

/*
 * DRV8871 input truth table:
 *   IN1 = 0,   IN2 = 0   -> coast (outputs Hi-Z)
 *   IN1 = 1,   IN2 = 1   -> brake (both outputs low)
 *   IN1 = PWM, IN2 = 0   -> forward at duty cycle
 *   IN1 = 0,   IN2 = PWM -> reverse at duty cycle
 */
static esp_err_t drv8871_apply(drv8871_ctx_t *ctx)
{
    esp_err_t err;
    uint32_t  duty     = (ctx->speed_pct * DRV8871_PWM_MAX_DUTY) / 100;
    uint32_t  in1_duty = 0;
    uint32_t  in2_duty = 0;

    if (ctx->braking)
    {
        in1_duty = DRV8871_PWM_MAX_DUTY;
        in2_duty = DRV8871_PWM_MAX_DUTY;
    }
    else if (ctx->direction == DRV8871_DIRECTION_FORWARD)
    {
        in1_duty = duty;
    }
    else
    {
        in2_duty = duty;
    }

    err = drv8871_set_duty(ctx, ctx->in1_pwm_channel, in1_duty);
    if (err != ESP_OK)
    {
        return err;
    }

    return drv8871_set_duty(ctx, ctx->in2_pwm_channel, in2_duty);
}

static esp_err_t drv8871_set_duty(drv8871_ctx_t *ctx, ledc_channel_t channel, uint32_t duty)
{
    esp_err_t err;

    err = ledc_set_duty(ctx->pwm_mode, channel, duty);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set duty (%s)", esp_err_to_name(err));
        return err;
    }

    err = ledc_update_duty(ctx->pwm_mode, channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update duty (%s)", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}
