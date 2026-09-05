#include "vsense.h"

#include <stdlib.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define VSENSE_DEFAULT_SAMPLE_COUNT  16
#define VSENSE_DEFAULT_HYSTERESIS_MV 200
#define VSENSE_MAX_SAMPLE_COUNT      256

/* Keeps the gain trim a correction rather than a way to invent a divider */
#define VSENSE_MIN_SCALE_PPM (VSENSE_SCALE_UNITY_PPM / 2)
#define VSENSE_MAX_SCALE_PPM (VSENSE_SCALE_UNITY_PPM * 2)

#define VSENSE_ADC_BITWIDTH ADC_BITWIDTH_12
#define VSENSE_ADC_MAX_RAW  4095

/* Used only when the chip carries no ADC calibration data in eFuse */
#define VSENSE_ADC_FULL_SCALE_MV 3100

typedef struct vsense_t
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t         cali_handle;
    adc_channel_t             adc_channel;

    uint32_t divider_sum_ohms; /* high side + low side */
    uint32_t low_side_ohms;
    uint32_t sample_count;
    uint32_t scale_ppm;
    uint32_t undervoltage_mv;
    uint32_t overvoltage_mv;
    uint32_t hysteresis_mv;
    bool     in_range;
} vsense_ctx_t;

static const char *TAG = "vsense";

static esp_err_t vsense_init_adc(vsense_ctx_t *ctx, adc_oneshot_unit_handle_t adc_unit, int in_gpio_num);
static esp_err_t vsense_init_calibration(vsense_ctx_t *ctx);
static esp_err_t vsense_sample_millivolts(vsense_ctx_t *ctx, int *out_millivolts, bool *out_clipped);
static uint32_t  vsense_pin_to_rail_mv(const vsense_ctx_t *ctx, int pin_millivolts);
static bool      vsense_update_range(vsense_ctx_t *ctx, uint32_t millivolts);
static bool      vsense_range_is_valid(uint32_t undervoltage_mv, uint32_t overvoltage_mv, uint32_t hysteresis_mv);

esp_err_t vsense_init(const vsense_config_t *config, vsense_handle_t *out_handle)
{
    esp_err_t     err;
    vsense_ctx_t *ctx;

    if (!config || !config->adc_unit || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->low_side_ohms == 0 || config->sample_count > VSENSE_MAX_SAMPLE_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->high_side_ohms > UINT32_MAX - config->low_side_ohms)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!vsense_range_is_valid(config->undervoltage_mv, config->overvoltage_mv,
                               config->hysteresis_mv ? config->hysteresis_mv : VSENSE_DEFAULT_HYSTERESIS_MV))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(vsense_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->divider_sum_ohms = config->high_side_ohms + config->low_side_ohms;
    ctx->low_side_ohms    = config->low_side_ohms;
    ctx->sample_count     = config->sample_count ? config->sample_count : VSENSE_DEFAULT_SAMPLE_COUNT;
    ctx->scale_ppm        = VSENSE_SCALE_UNITY_PPM;
    ctx->undervoltage_mv  = config->undervoltage_mv;
    ctx->overvoltage_mv   = config->overvoltage_mv;
    ctx->hysteresis_mv    = config->hysteresis_mv ? config->hysteresis_mv : VSENSE_DEFAULT_HYSTERESIS_MV;

    /* Assume the rail is healthy until a reading says otherwise */
    ctx->in_range = true;

    err = vsense_init_adc(ctx, config->adc_unit, config->in_gpio_num);
    if (err != ESP_OK)
    {
        free(ctx);
        return err;
    }

    /* Calibration is a refinement; an uncalibrated chip still gives usable readings */
    if (vsense_init_calibration(ctx) != ESP_OK)
    {
        ESP_LOGW(TAG, "No ADC calibration available, falling back to a nominal %d mV full scale",
                 VSENSE_ADC_FULL_SCALE_MV);
    }

    ESP_LOGI(TAG, "Initialized (tap on GPIO%d, %lu:%lu divider, up to %lu mV)", config->in_gpio_num,
             (unsigned long) (ctx->divider_sum_ohms - ctx->low_side_ohms), (unsigned long) ctx->low_side_ohms,
             (unsigned long) vsense_pin_to_rail_mv(ctx, VSENSE_ADC_FULL_SCALE_MV));

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t vsense_read(vsense_handle_t handle, vsense_reading_t *out_reading)
{
    esp_err_t err;
    int       pin_millivolts;
    bool      clipped;

    if (!handle || !out_reading)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = vsense_sample_millivolts(handle, &pin_millivolts, &clipped);
    if (err != ESP_OK)
    {
        return err;
    }

    out_reading->pin_millivolts = pin_millivolts;
    out_reading->millivolts     = vsense_pin_to_rail_mv(handle, pin_millivolts);
    out_reading->clipped        = clipped;
    out_reading->in_range       = vsense_update_range(handle, out_reading->millivolts);

    return ESP_OK;
}

esp_err_t vsense_read_millivolts(vsense_handle_t handle, uint32_t *out_millivolts)
{
    esp_err_t        err;
    vsense_reading_t reading;

    if (!handle || !out_millivolts)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = vsense_read(handle, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_millivolts = reading.millivolts;
    return ESP_OK;
}

esp_err_t vsense_is_in_range(vsense_handle_t handle, bool *out_in_range)
{
    esp_err_t        err;
    vsense_reading_t reading;

    if (!handle || !out_in_range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = vsense_read(handle, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_in_range = reading.in_range;
    return ESP_OK;
}

esp_err_t vsense_get_max_millivolts(vsense_handle_t handle, uint32_t *out_millivolts)
{
    if (!handle || !out_millivolts)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_millivolts = vsense_pin_to_rail_mv(handle, VSENSE_ADC_FULL_SCALE_MV);
    return ESP_OK;
}

esp_err_t vsense_calibrate(vsense_handle_t handle, uint32_t actual_millivolts)
{
    esp_err_t err;
    int       pin_millivolts;
    uint32_t  nominal_mv;
    uint64_t  scale_ppm;
    bool      clipped;

    if (!handle || actual_millivolts == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = vsense_sample_millivolts(handle, &pin_millivolts, &clipped);
    if (err != ESP_OK)
    {
        return err;
    }

    if (clipped)
    {
        ESP_LOGE(TAG, "Cannot calibrate, the ADC is saturated");
        return ESP_ERR_INVALID_STATE;
    }

    /* Trim against the uncorrected reading so repeated calibrations do not compound */
    nominal_mv = (uint32_t) (((uint64_t) (pin_millivolts < 0 ? 0 : pin_millivolts) * handle->divider_sum_ohms) /
                             handle->low_side_ohms);
    if (nominal_mv == 0)
    {
        ESP_LOGE(TAG, "Cannot calibrate, the tap reads 0 mV");
        return ESP_ERR_INVALID_STATE;
    }

    scale_ppm = ((uint64_t) actual_millivolts * VSENSE_SCALE_UNITY_PPM) / nominal_mv;

    if (scale_ppm < VSENSE_MIN_SCALE_PPM || scale_ppm > VSENSE_MAX_SCALE_PPM)
    {
        ESP_LOGE(TAG, "Cannot calibrate, %lu mV against a %lu mV reading is not a trim",
                 (unsigned long) actual_millivolts, (unsigned long) nominal_mv);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Gain correction moved from %lu ppm to %lu ppm", (unsigned long) handle->scale_ppm,
             (unsigned long) scale_ppm);

    handle->scale_ppm = (uint32_t) scale_ppm;
    handle->in_range  = true;

    return ESP_OK;
}

esp_err_t vsense_get_scale_ppm(vsense_handle_t handle, uint32_t *out_scale_ppm)
{
    if (!handle || !out_scale_ppm)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_scale_ppm = handle->scale_ppm;
    return ESP_OK;
}

esp_err_t vsense_set_scale_ppm(vsense_handle_t handle, uint32_t scale_ppm)
{
    if (!handle || scale_ppm < VSENSE_MIN_SCALE_PPM || scale_ppm > VSENSE_MAX_SCALE_PPM)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->scale_ppm = scale_ppm;
    handle->in_range  = true;

    return ESP_OK;
}

esp_err_t vsense_set_range(vsense_handle_t handle, uint32_t undervoltage_mv, uint32_t overvoltage_mv,
                           uint32_t hysteresis_mv)
{
    if (!handle || !vsense_range_is_valid(undervoltage_mv, overvoltage_mv, hysteresis_mv))
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->undervoltage_mv = undervoltage_mv;
    handle->overvoltage_mv  = overvoltage_mv;
    handle->hysteresis_mv   = hysteresis_mv;
    handle->in_range        = true;

    return ESP_OK;
}

esp_err_t vsense_get_range(vsense_handle_t handle, uint32_t *out_undervoltage_mv, uint32_t *out_overvoltage_mv,
                           uint32_t *out_hysteresis_mv)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (out_undervoltage_mv)
    {
        *out_undervoltage_mv = handle->undervoltage_mv;
    }

    if (out_overvoltage_mv)
    {
        *out_overvoltage_mv = handle->overvoltage_mv;
    }

    if (out_hysteresis_mv)
    {
        *out_hysteresis_mv = handle->hysteresis_mv;
    }

    return ESP_OK;
}

static esp_err_t vsense_init_adc(vsense_ctx_t *ctx, adc_oneshot_unit_handle_t adc_unit, int in_gpio_num)
{
    esp_err_t  err;
    adc_unit_t unit;

    err = adc_oneshot_io_to_channel(in_gpio_num, &unit, &ctx->adc_channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO%d is not an ADC pin (%s)", in_gpio_num, esp_err_to_name(err));
        return ESP_ERR_INVALID_ARG;
    }

    /* ADC2 is unusable while Wi-Fi runs, so the divider tap has to sit on ADC1 */
    if (unit != ADC_UNIT_1)
    {
        ESP_LOGE(TAG, "GPIO%d is on ADC%d, only ADC1 is supported", in_gpio_num, unit + 1);
        return ESP_ERR_INVALID_ARG;
    }

    ctx->adc_handle = adc_unit;

    /* 12 dB is the widest input range the ADC offers, so the divider can stay lightly loaded */
    adc_oneshot_chan_cfg_t channel_config = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = VSENSE_ADC_BITWIDTH,
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

static esp_err_t vsense_init_calibration(vsense_ctx_t *ctx)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ctx->adc_channel,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = VSENSE_ADC_BITWIDTH,
    };

    return adc_cali_create_scheme_curve_fitting(&cali_config, &ctx->cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = VSENSE_ADC_BITWIDTH,
    };

    return adc_cali_create_scheme_line_fitting(&cali_config, &ctx->cali_handle);
#else
    ctx->cali_handle = NULL;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/*
 * Averaging happens on raw counts rather than on converted millivolts so that
 * the calibration curve is only evaluated once per reading. A single saturated
 * sample is enough to call the reading clipped: the average would otherwise
 * hide it behind the samples that were still in range.
 */
static esp_err_t vsense_sample_millivolts(vsense_ctx_t *ctx, int *out_millivolts, bool *out_clipped)
{
    esp_err_t err;
    int32_t   sum     = 0;
    bool      clipped = false;
    int       raw;

    for (uint32_t i = 0; i < ctx->sample_count; i++)
    {
        err = adc_oneshot_read(ctx->adc_handle, ctx->adc_channel, &raw);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read the ADC (%s)", esp_err_to_name(err));
            return err;
        }

        if (raw >= VSENSE_ADC_MAX_RAW)
        {
            clipped = true;
        }

        sum += raw;
    }

    raw          = (int) (sum / (int32_t) ctx->sample_count);
    *out_clipped = clipped;

    if (ctx->cali_handle)
    {
        return adc_cali_raw_to_voltage(ctx->cali_handle, raw, out_millivolts);
    }

    *out_millivolts = (raw * VSENSE_ADC_FULL_SCALE_MV) / VSENSE_ADC_MAX_RAW;
    return ESP_OK;
}

/* Scale the tap voltage back up through the divider, then apply the gain trim */
static uint32_t vsense_pin_to_rail_mv(const vsense_ctx_t *ctx, int pin_millivolts)
{
    uint64_t rail_mv;

    if (pin_millivolts <= 0)
    {
        return 0;
    }

    rail_mv = ((uint64_t) pin_millivolts * ctx->divider_sum_ohms) / ctx->low_side_ohms;
    rail_mv = (rail_mv * ctx->scale_ppm) / VSENSE_SCALE_UNITY_PPM;

    /* An absurd divider ratio must not wrap the range comparisons */
    return rail_mv > UINT32_MAX / 2 ? UINT32_MAX / 2 : (uint32_t) rail_mv;
}

/*
 * Leaving the window takes a single reading past a limit, coming back takes a
 * reading clear of it by the hysteresis. A limit set to 0 is disabled.
 */
static bool vsense_update_range(vsense_ctx_t *ctx, uint32_t millivolts)
{
    uint32_t margin = ctx->in_range ? 0 : ctx->hysteresis_mv;

    bool above_floor = ctx->undervoltage_mv == 0 || millivolts >= ctx->undervoltage_mv + margin;
    bool below_ceil  = ctx->overvoltage_mv == 0 || millivolts + margin <= ctx->overvoltage_mv;

    ctx->in_range = above_floor && below_ceil;
    return ctx->in_range;
}

/* Both limits may be disabled, but a window that is set has to be wider than the hysteresis */
static bool vsense_range_is_valid(uint32_t undervoltage_mv, uint32_t overvoltage_mv, uint32_t hysteresis_mv)
{
    if (undervoltage_mv == 0 || overvoltage_mv == 0)
    {
        return true;
    }

    return overvoltage_mv > undervoltage_mv && overvoltage_mv - undervoltage_mv > hysteresis_mv;
}
