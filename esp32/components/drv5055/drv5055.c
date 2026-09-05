#include "drv5055.h"

#include <stdlib.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define DRV5055_DEFAULT_SUPPLY_MV    3300
#define DRV5055_DEFAULT_SAMPLE_COUNT 16
#define DRV5055_DEFAULT_THRESHOLD_UT 5000
#define DRV5055_MAX_SAMPLE_COUNT     256

/* The datasheet guarantees a linear output between 0.2 V and VCC - 0.2 V */
#define DRV5055_LINEAR_MARGIN_MV 200

/* Used only when the chip carries no ADC calibration data in eFuse */
#define DRV5055_ADC_BITWIDTH      ADC_BITWIDTH_12
#define DRV5055_ADC_MAX_RAW       4095
#define DRV5055_ADC_FULL_SCALE_MV 3100

typedef struct drv5055_t
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t         cali_handle;
    adc_channel_t             adc_channel;

    int32_t  sensitivity_uv_mt;
    uint32_t supply_mv;
    uint32_t sample_count;
    int32_t  threshold_ut;
    int32_t  hysteresis_ut;
    int      zero_mv;
    bool     magnet_present;
} drv5055_ctx_t;

static const char *TAG = "drv5055";

static esp_err_t drv5055_init_adc(drv5055_ctx_t *ctx, adc_oneshot_unit_handle_t adc_unit, int out_gpio_num);
static esp_err_t drv5055_init_calibration(drv5055_ctx_t *ctx);
static esp_err_t drv5055_sample_millivolts(drv5055_ctx_t *ctx, int *out_millivolts);
static int32_t   drv5055_millivolts_to_field_ut(const drv5055_ctx_t *ctx, int millivolts);
static bool      drv5055_update_presence(drv5055_ctx_t *ctx, int32_t field_ut);

esp_err_t drv5055_init(const drv5055_config_t *config, drv5055_handle_t *out_handle)
{
    esp_err_t      err;
    drv5055_ctx_t *ctx;

    if (!config || !config->adc_unit || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->sensitivity_uv_mt < 0 || config->threshold_ut < 0 || config->hysteresis_ut < 0 ||
        config->sample_count > DRV5055_MAX_SAMPLE_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(drv5055_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->sensitivity_uv_mt = config->sensitivity_uv_mt ? config->sensitivity_uv_mt : DRV5055_SENSITIVITY_A3_3V3_UV_MT;
    ctx->supply_mv         = config->supply_mv ? config->supply_mv : DRV5055_DEFAULT_SUPPLY_MV;
    ctx->sample_count      = config->sample_count ? config->sample_count : DRV5055_DEFAULT_SAMPLE_COUNT;
    ctx->threshold_ut      = config->threshold_ut ? config->threshold_ut : DRV5055_DEFAULT_THRESHOLD_UT;
    ctx->hysteresis_ut     = config->hysteresis_ut ? config->hysteresis_ut : ctx->threshold_ut / 4;
    ctx->zero_mv           = (int) (ctx->supply_mv / 2);
    ctx->magnet_present    = false;

    if (ctx->hysteresis_ut >= ctx->threshold_ut)
    {
        free(ctx);
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_init_adc(ctx, config->adc_unit, config->out_gpio_num);
    if (err != ESP_OK)
    {
        free(ctx);
        return err;
    }

    /* Calibration is a refinement; an uncalibrated chip still gives usable readings */
    if (drv5055_init_calibration(ctx) != ESP_OK)
    {
        ESP_LOGW(TAG, "No ADC calibration available, falling back to a nominal %d mV full scale",
                 DRV5055_ADC_FULL_SCALE_MV);
    }

    ESP_LOGI(TAG, "Initialized (OUT on GPIO%d, %ld uV/mT, zero at %d mV)", config->out_gpio_num,
             (long) ctx->sensitivity_uv_mt, ctx->zero_mv);

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t drv5055_read(drv5055_handle_t handle, drv5055_reading_t *out_reading)
{
    esp_err_t err;
    int       millivolts;

    if (!handle || !out_reading)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_sample_millivolts(handle, &millivolts);
    if (err != ESP_OK)
    {
        return err;
    }

    out_reading->millivolts = millivolts;
    out_reading->field_ut   = drv5055_millivolts_to_field_ut(handle, millivolts);
    out_reading->saturated =
        millivolts <= DRV5055_LINEAR_MARGIN_MV || millivolts >= (int) handle->supply_mv - DRV5055_LINEAR_MARGIN_MV;
    out_reading->magnet_present = drv5055_update_presence(handle, out_reading->field_ut);

    return ESP_OK;
}

esp_err_t drv5055_read_field_ut(drv5055_handle_t handle, int32_t *out_field_ut)
{
    esp_err_t         err;
    drv5055_reading_t reading;

    if (!handle || !out_field_ut)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_read(handle, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_field_ut = reading.field_ut;
    return ESP_OK;
}

esp_err_t drv5055_read_millivolts(drv5055_handle_t handle, int *out_millivolts)
{
    if (!handle || !out_millivolts)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return drv5055_sample_millivolts(handle, out_millivolts);
}

esp_err_t drv5055_is_magnet_present(drv5055_handle_t handle, bool *out_present)
{
    esp_err_t         err;
    drv5055_reading_t reading;

    if (!handle || !out_present)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_read(handle, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_present = reading.magnet_present;
    return ESP_OK;
}

esp_err_t drv5055_calibrate_zero(drv5055_handle_t handle)
{
    esp_err_t err;
    int       millivolts;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_sample_millivolts(handle, &millivolts);
    if (err != ESP_OK)
    {
        return err;
    }

    ESP_LOGI(TAG, "Zero-field reference moved from %d mV to %d mV", handle->zero_mv, millivolts);

    handle->zero_mv        = millivolts;
    handle->magnet_present = false;

    return ESP_OK;
}

esp_err_t drv5055_get_zero_millivolts(drv5055_handle_t handle, int *out_millivolts)
{
    if (!handle || !out_millivolts)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_millivolts = handle->zero_mv;
    return ESP_OK;
}

esp_err_t drv5055_set_zero_millivolts(drv5055_handle_t handle, int millivolts)
{
    if (!handle || millivolts < 0 || millivolts > (int) handle->supply_mv)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->zero_mv        = millivolts;
    handle->magnet_present = false;

    return ESP_OK;
}

esp_err_t drv5055_set_threshold(drv5055_handle_t handle, int32_t threshold_ut, int32_t hysteresis_ut)
{
    if (!handle || threshold_ut <= 0 || hysteresis_ut < 0 || hysteresis_ut >= threshold_ut)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->threshold_ut   = threshold_ut;
    handle->hysteresis_ut  = hysteresis_ut;
    handle->magnet_present = false;

    return ESP_OK;
}

esp_err_t drv5055_get_threshold(drv5055_handle_t handle, int32_t *out_threshold_ut, int32_t *out_hysteresis_ut)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (out_threshold_ut)
    {
        *out_threshold_ut = handle->threshold_ut;
    }

    if (out_hysteresis_ut)
    {
        *out_hysteresis_ut = handle->hysteresis_ut;
    }

    return ESP_OK;
}

static esp_err_t drv5055_init_adc(drv5055_ctx_t *ctx, adc_oneshot_unit_handle_t adc_unit, int out_gpio_num)
{
    esp_err_t  err;
    adc_unit_t unit;

    err = adc_oneshot_io_to_channel(out_gpio_num, &unit, &ctx->adc_channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO%d is not an ADC pin (%s)", out_gpio_num, esp_err_to_name(err));
        return ESP_ERR_INVALID_ARG;
    }

    /* ADC2 is unusable while Wi-Fi runs, so the OUT pin has to sit on ADC1 */
    if (unit != ADC_UNIT_1)
    {
        ESP_LOGE(TAG, "GPIO%d is on ADC%d, only ADC1 is supported", out_gpio_num, unit + 1);
        return ESP_ERR_INVALID_ARG;
    }

    ctx->adc_handle = adc_unit;

    /*
     * 12 dB keeps the whole VCC/2 +/- full-scale swing inside the ADC range;
     * anything less clips before the sensor leaves its linear window.
     */
    adc_oneshot_chan_cfg_t channel_config = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = DRV5055_ADC_BITWIDTH,
    };

    err = adc_oneshot_config_channel(ctx->adc_handle, ctx->adc_channel, &channel_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure the ADC channel (%s)", esp_err_to_name(err));
        ctx->adc_handle = NULL;
        return err;
    }

    return ESP_OK;
}

static esp_err_t drv5055_init_calibration(drv5055_ctx_t *ctx)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ctx->adc_channel,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = DRV5055_ADC_BITWIDTH,
    };

    return adc_cali_create_scheme_curve_fitting(&cali_config, &ctx->cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = DRV5055_ADC_BITWIDTH,
    };

    return adc_cali_create_scheme_line_fitting(&cali_config, &ctx->cali_handle);
#else
    ctx->cali_handle = NULL;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/*
 * Averaging happens on raw counts rather than on converted millivolts so that
 * the calibration curve is only evaluated once per reading.
 */
static esp_err_t drv5055_sample_millivolts(drv5055_ctx_t *ctx, int *out_millivolts)
{
    esp_err_t err;
    int32_t   sum = 0;
    int       raw;

    for (uint32_t i = 0; i < ctx->sample_count; i++)
    {
        err = adc_oneshot_read(ctx->adc_handle, ctx->adc_channel, &raw);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read the ADC (%s)", esp_err_to_name(err));
            return err;
        }

        sum += raw;
    }

    raw = (int) (sum / (int32_t) ctx->sample_count);

    if (ctx->cali_handle)
    {
        return adc_cali_raw_to_voltage(ctx->cali_handle, raw, out_millivolts);
    }

    *out_millivolts = (raw * DRV5055_ADC_FULL_SCALE_MV) / DRV5055_ADC_MAX_RAW;
    return ESP_OK;
}

/*
 * VOUT = VQ + B * Sensitivity, so B is the distance from the zero-field
 * reference divided by the sensitivity. The datasheet sensitivity temperature
 * coefficient of 0.12 %/degC is not compensated for here; over a 40 degC swing
 * that is about 5 % of reading, which threshold detection tolerates.
 */
static int32_t drv5055_millivolts_to_field_ut(const drv5055_ctx_t *ctx, int millivolts)
{
    int64_t delta_uv = (int64_t) (millivolts - ctx->zero_mv) * 1000;

    return (int32_t) ((delta_uv * 1000) / ctx->sensitivity_uv_mt);
}

static bool drv5055_update_presence(drv5055_ctx_t *ctx, int32_t field_ut)
{
    int32_t magnitude = field_ut < 0 ? -field_ut : field_ut;

    if (ctx->magnet_present)
    {
        if (magnitude < ctx->threshold_ut - ctx->hysteresis_ut)
        {
            ctx->magnet_present = false;
        }
    }
    else if (magnitude > ctx->threshold_ut + ctx->hysteresis_ut)
    {
        ctx->magnet_present = true;
    }

    return ctx->magnet_present;
}
