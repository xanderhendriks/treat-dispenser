#include "ch224a.h"

#include <stdlib.h>

#include "esp_log.h"

#define CH224A_I2C_DEFAULT_SPEED_HZ 400000
#define CH224A_I2C_TIMEOUT_MS       100

typedef struct ch224a_t
{
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;

    uint8_t          address;
    ch224a_voltage_t max_voltage;
    ch224a_voltage_t requested_voltage;
    bool             voltage_requested;
} ch224a_ctx_t;

static const char *TAG = "ch224a";

static esp_err_t ch224a_add_device(ch224a_ctx_t *ctx, uint8_t address, uint32_t speed_hz);
static void      ch224a_remove_device(ch224a_ctx_t *ctx);

esp_err_t ch224a_init(const ch224a_config_t *config, ch224a_handle_t *out_handle)
{
    static const uint8_t candidates[] = {CH224A_I2C_ADDRESS_PRIMARY, CH224A_I2C_ADDRESS_SECONDARY};

    esp_err_t     err = ESP_ERR_NOT_FOUND;
    ch224a_ctx_t *ctx;
    uint32_t      speed_hz;
    size_t        candidate_count;
    uint8_t       status;

    if (!config || !config->bus || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->max_voltage > CH224A_VOLTAGE_28V)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(ch224a_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->bus         = config->bus;
    ctx->max_voltage = config->max_voltage;

    speed_hz        = config->i2c_clock_speed_hz ? config->i2c_clock_speed_hz : CH224A_I2C_DEFAULT_SPEED_HZ;
    candidate_count = config->i2c_address ? 1 : sizeof(candidates) / sizeof(candidates[0]);

    for (size_t i = 0; i < candidate_count; i++)
    {
        uint8_t address = config->i2c_address ? config->i2c_address : candidates[i];

        err = ch224a_add_device(ctx, address, speed_hz);
        if (err != ESP_OK)
        {
            break;
        }

        err = ch224a_read_register(ctx, CH224A_REG_STATUS, &status);
        if (err == ESP_OK)
        {
            ctx->address = address;
            break;
        }

        ESP_LOGD(TAG, "No answer at 0x%02X (%s)", address, esp_err_to_name(err));
        ch224a_remove_device(ctx);
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "CH224A not responding (%s); check that CFG1 is strapped to GND so I2C is enabled",
                 esp_err_to_name(err));
        free(ctx);
        return err == ESP_ERR_TIMEOUT ? err : ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Initialized at 0x%02X (status 0x%02X, max request %lu mV)", ctx->address, status,
             (unsigned long) ch224a_voltage_to_mv(ctx->max_voltage));

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t ch224a_get_address(ch224a_handle_t handle, uint8_t *out_address)
{
    if (!handle || !out_address)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_address = handle->address;
    return ESP_OK;
}

esp_err_t ch224a_read_register(ch224a_handle_t handle, uint8_t reg, uint8_t *out_value)
{
    if (!handle || !out_value)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(handle->dev, &reg, 1, out_value, 1, CH224A_I2C_TIMEOUT_MS);
}

esp_err_t ch224a_write_register(ch224a_handle_t handle, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    buf[0] = reg;
    buf[1] = value;

    return i2c_master_transmit(handle->dev, buf, sizeof(buf), CH224A_I2C_TIMEOUT_MS);
}

esp_err_t ch224a_read_registers(ch224a_handle_t handle, uint8_t first_reg, uint8_t *out_values, size_t count)
{
    if (!handle || !out_values)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (first_reg + count > 0x100)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < count; i++)
    {
        esp_err_t err = ch224a_read_register(handle, (uint8_t) (first_reg + i), &out_values[i]);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

esp_err_t ch224a_read_all_registers(ch224a_handle_t handle, ch224a_register_t *out_registers, size_t max_registers,
                                    size_t *out_count)
{
    static const uint8_t named_registers[] = {
        CH224A_REG_STATUS,   CH224A_REG_VOLTAGE, CH224A_REG_CURRENT,
        CH224A_REG_AVS_HIGH, CH224A_REG_AVS_LOW, CH224A_REG_PPS,
    };

    size_t count = 0;

    if (!handle || !out_registers || !out_count)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (max_registers == 0)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0; i < sizeof(named_registers) / sizeof(named_registers[0]) && count < max_registers; i++)
    {
        out_registers[count].address = named_registers[i];
        out_registers[count].value   = 0;
        out_registers[count].err     = ch224a_read_register(handle, named_registers[i], &out_registers[count].value);
        count++;
    }

    for (uint8_t reg = CH224A_REG_PD_DATA_FIRST; reg <= CH224A_REG_PD_DATA_LAST && count < max_registers; reg++)
    {
        out_registers[count].address = reg;
        out_registers[count].value   = 0;
        out_registers[count].err     = ch224a_read_register(handle, reg, &out_registers[count].value);
        count++;
    }

    *out_count = count;
    return ESP_OK;
}

const char *ch224a_register_name(uint8_t reg)
{
    switch (reg)
    {
        case CH224A_REG_STATUS:
            return "status";
        case CH224A_REG_VOLTAGE:
            return "voltage control";
        case CH224A_REG_CURRENT:
            return "current data";
        case CH224A_REG_AVS_HIGH:
            return "AVS voltage high";
        case CH224A_REG_AVS_LOW:
            return "AVS voltage low";
        case CH224A_REG_PPS:
            return "PPS voltage";
        default:
            break;
    }

    if (reg >= CH224A_REG_PD_DATA_FIRST && reg <= CH224A_REG_PD_DATA_LAST)
    {
        return "PD power data";
    }

    return "reserved";
}

esp_err_t ch224a_get_status(ch224a_handle_t handle, uint8_t *out_status)
{
    return ch224a_read_register(handle, CH224A_REG_STATUS, out_status);
}

esp_err_t ch224a_get_max_current_ma(ch224a_handle_t handle, uint16_t *out_current_ma)
{
    esp_err_t err;
    uint8_t   raw;

    if (!out_current_ma)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = ch224a_read_register(handle, CH224A_REG_CURRENT, &raw);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_current_ma = (uint16_t) raw * CH224A_CURRENT_STEP_MA;
    return ESP_OK;
}

esp_err_t ch224a_set_voltage(ch224a_handle_t handle, ch224a_voltage_t voltage)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (voltage > handle->max_voltage)
    {
        ESP_LOGE(TAG, "Refusing %lu mV request, board limit is %lu mV", (unsigned long) ch224a_voltage_to_mv(voltage),
                 (unsigned long) ch224a_voltage_to_mv(handle->max_voltage));
        return ESP_ERR_INVALID_ARG;
    }

    err = ch224a_write_register(handle, CH224A_REG_VOLTAGE, (uint8_t) voltage);
    if (err != ESP_OK)
    {
        return err;
    }

    handle->requested_voltage = voltage;
    handle->voltage_requested = true;

    return ESP_OK;
}

esp_err_t ch224a_get_requested_voltage(ch224a_handle_t handle, ch224a_voltage_t *out_voltage)
{
    if (!handle || !out_voltage)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->voltage_requested)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *out_voltage = handle->requested_voltage;
    return ESP_OK;
}

uint32_t ch224a_voltage_to_mv(ch224a_voltage_t voltage)
{
    switch (voltage)
    {
        case CH224A_VOLTAGE_5V:
            return 5000;
        case CH224A_VOLTAGE_9V:
            return 9000;
        case CH224A_VOLTAGE_12V:
            return 12000;
        case CH224A_VOLTAGE_15V:
            return 15000;
        case CH224A_VOLTAGE_20V:
            return 20000;
        case CH224A_VOLTAGE_28V:
            return 28000;
        default:
            return 0;
    }
}

static esp_err_t ch224a_add_device(ch224a_ctx_t *ctx, uint8_t address, uint32_t speed_hz)
{
    esp_err_t err;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = address,
        .scl_speed_hz    = speed_hz,
    };

    err = i2c_master_bus_add_device(ctx->bus, &dev_config, &ctx->dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X (%s)", address, esp_err_to_name(err));
        ctx->dev = NULL;
    }

    return err;
}

static void ch224a_remove_device(ch224a_ctx_t *ctx)
{
    if (ctx->dev)
    {
        i2c_master_bus_rm_device(ctx->dev);
        ctx->dev = NULL;
    }
}
