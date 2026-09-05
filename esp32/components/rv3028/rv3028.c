#include "rv3028.h"

#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RV3028_I2C_ADDRESS          0x52
#define RV3028_I2C_DEFAULT_SPEED_HZ 400000
#define RV3028_I2C_TIMEOUT_MS       100

/* Register addresses */
#define RV3028_REG_SECONDS       0x00
#define RV3028_REG_MINUTES       0x01
#define RV3028_REG_HOURS         0x02
#define RV3028_REG_WEEKDAY       0x03
#define RV3028_REG_DATE          0x04
#define RV3028_REG_MONTH         0x05
#define RV3028_REG_YEAR          0x06
#define RV3028_REG_STATUS        0x0E
#define RV3028_REG_CONTROL1      0x0F
#define RV3028_REG_EE_COMMAND    0x27
#define RV3028_REG_EEPROM_BACKUP 0x37

/* Status register bit masks */
#define RV3028_STATUS_PORF   0x01
#define RV3028_STATUS_EEBUSY 0x80

/* Control 1 register bit masks */
#define RV3028_CONTROL1_EERD 0x08

/* EE command register values */
#define RV3028_EE_COMMAND_FIRST  0x00
#define RV3028_EE_COMMAND_UPDATE 0x11

/* EEPROM backup register bit masks */
#define RV3028_BACKUP_TCE_MASK  0x20
#define RV3028_BACKUP_BSM_MASK  0x0C
#define RV3028_BACKUP_BSM_LEVEL 0x0C
#define RV3028_BACKUP_TCR_MASK  0x03

#define RV3028_EEBUSY_TIMEOUT_MS 100

typedef struct rv3028_t
{
    i2c_master_dev_handle_t dev;
} rv3028_ctx_t;

static const char *TAG = "rv3028";

static esp_err_t rv3028_read_regs(rv3028_ctx_t *ctx, uint8_t reg, uint8_t *data, size_t len);
static esp_err_t rv3028_write_regs(rv3028_ctx_t *ctx, uint8_t reg, const uint8_t *data, size_t len);
static esp_err_t rv3028_configure_backup(rv3028_ctx_t *ctx);
static esp_err_t rv3028_wait_eeprom_ready(rv3028_ctx_t *ctx);
static uint8_t   rv3028_to_bcd(int value);
static int       rv3028_from_bcd(uint8_t value);
static int       rv3028_weekday(int year, int month, int day);

esp_err_t rv3028_init(const rv3028_config_t *config, rv3028_handle_t *out_handle)
{
    esp_err_t     err;
    rv3028_ctx_t *ctx;
    uint8_t       status;

    if (!config || !config->bus || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(rv3028_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = RV3028_I2C_ADDRESS,
        .scl_speed_hz    = config->i2c_clock_speed_hz ? config->i2c_clock_speed_hz : RV3028_I2C_DEFAULT_SPEED_HZ,
    };

    err = i2c_master_bus_add_device(config->bus, &dev_config, &ctx->dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device (%s)", esp_err_to_name(err));
        free(ctx);
        return err;
    }

    err = rv3028_read_regs(ctx, RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "RTC not responding at 0x%02X (%s), run i2c_scan to see what is on the bus", RV3028_I2C_ADDRESS,
                 esp_err_to_name(err));
        i2c_master_bus_rm_device(ctx->dev);
        free(ctx);
        return err;
    }

    if (status & RV3028_STATUS_PORF)
    {
        ESP_LOGW(TAG, "Power-on reset detected, RTC time is invalid");
    }

    err = rv3028_configure_backup(ctx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure backup switchover (%s)", esp_err_to_name(err));
        i2c_master_bus_rm_device(ctx->dev);
        free(ctx);
        return err;
    }

    ESP_LOGI(TAG, "Initialized at 0x%02X", RV3028_I2C_ADDRESS);

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t rv3028_set_time(rv3028_handle_t handle, const struct tm *time)
{
    esp_err_t err;
    uint8_t   regs[7];
    uint8_t   status;

    if (!handle || !time)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int year  = time->tm_year + 1900;
    int month = time->tm_mon + 1;

    if (year < 2000 || year > 2099 || month < 1 || month > 12 || time->tm_mday < 1 || time->tm_mday > 31 ||
        time->tm_hour < 0 || time->tm_hour > 23 || time->tm_min < 0 || time->tm_min > 59 || time->tm_sec < 0 ||
        time->tm_sec > 59)
    {
        return ESP_ERR_INVALID_ARG;
    }

    regs[0] = rv3028_to_bcd(time->tm_sec);
    regs[1] = rv3028_to_bcd(time->tm_min);
    regs[2] = rv3028_to_bcd(time->tm_hour);
    regs[3] = (uint8_t) rv3028_weekday(year, month, time->tm_mday);
    regs[4] = rv3028_to_bcd(time->tm_mday);
    regs[5] = rv3028_to_bcd(month);
    regs[6] = rv3028_to_bcd(year - 2000);

    err = rv3028_write_regs(handle, RV3028_REG_SECONDS, regs, sizeof(regs));
    if (err != ESP_OK)
    {
        return err;
    }

    /* Clear the power-on reset flag so the time reads as valid again */
    err = rv3028_read_regs(handle, RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK)
    {
        return err;
    }

    if (status & RV3028_STATUS_PORF)
    {
        status &= ~RV3028_STATUS_PORF;
        err = rv3028_write_regs(handle, RV3028_REG_STATUS, &status, 1);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

esp_err_t rv3028_get_time(rv3028_handle_t handle, struct tm *out_time)
{
    esp_err_t err;
    uint8_t   regs[7];

    if (!handle || !out_time)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = rv3028_read_regs(handle, RV3028_REG_SECONDS, regs, sizeof(regs));
    if (err != ESP_OK)
    {
        return err;
    }

    out_time->tm_sec   = rv3028_from_bcd(regs[0] & 0x7F);
    out_time->tm_min   = rv3028_from_bcd(regs[1] & 0x7F);
    out_time->tm_hour  = rv3028_from_bcd(regs[2] & 0x3F);
    out_time->tm_wday  = regs[3] & 0x07;
    out_time->tm_mday  = rv3028_from_bcd(regs[4] & 0x3F);
    out_time->tm_mon   = rv3028_from_bcd(regs[5] & 0x1F) - 1;
    out_time->tm_year  = rv3028_from_bcd(regs[6]) + 100;
    out_time->tm_yday  = 0;
    out_time->tm_isdst = -1;

    return ESP_OK;
}

esp_err_t rv3028_is_time_valid(rv3028_handle_t handle, bool *out_valid)
{
    esp_err_t err;
    uint8_t   status;

    if (!handle || !out_valid)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = rv3028_read_regs(handle, RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_valid = (status & RV3028_STATUS_PORF) == 0;
    return ESP_OK;
}

static esp_err_t rv3028_read_regs(rv3028_ctx_t *ctx, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(ctx->dev, &reg, 1, data, len, RV3028_I2C_TIMEOUT_MS);
}

static esp_err_t rv3028_write_regs(rv3028_ctx_t *ctx, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[8];

    if (len > sizeof(buf) - 1)
    {
        return ESP_ERR_INVALID_ARG;
    }

    buf[0] = reg;
    for (size_t i = 0; i < len; i++)
    {
        buf[i + 1] = data[i];
    }

    return i2c_master_transmit(ctx->dev, buf, len + 1, RV3028_I2C_TIMEOUT_MS);
}

/*
 * Enable backup power switchover in level switching mode and keep trickle
 * charging disabled (the backup battery is a non-rechargeable coin cell).
 * The setting lives in EEPROM; only update it when it differs to avoid
 * unnecessary EEPROM wear.
 */
static esp_err_t rv3028_configure_backup(rv3028_ctx_t *ctx)
{
    esp_err_t err;
    uint8_t   backup;
    uint8_t   control1;
    uint8_t   command;

    err = rv3028_read_regs(ctx, RV3028_REG_EEPROM_BACKUP, &backup, 1);
    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t desired = backup;
    desired &= ~(RV3028_BACKUP_TCE_MASK | RV3028_BACKUP_BSM_MASK | RV3028_BACKUP_TCR_MASK);
    desired |= RV3028_BACKUP_BSM_LEVEL;

    if (desired == backup)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Updating backup configuration (0x%02X -> 0x%02X)", backup, desired);

    /* Disable automatic EEPROM refresh while updating */
    err = rv3028_read_regs(ctx, RV3028_REG_CONTROL1, &control1, 1);
    if (err != ESP_OK)
    {
        return err;
    }

    control1 |= RV3028_CONTROL1_EERD;
    err = rv3028_write_regs(ctx, RV3028_REG_CONTROL1, &control1, 1);
    if (err != ESP_OK)
    {
        return err;
    }

    err = rv3028_wait_eeprom_ready(ctx);
    if (err == ESP_OK)
    {
        err = rv3028_write_regs(ctx, RV3028_REG_EEPROM_BACKUP, &desired, 1);
    }

    if (err == ESP_OK)
    {
        command = RV3028_EE_COMMAND_FIRST;
        err     = rv3028_write_regs(ctx, RV3028_REG_EE_COMMAND, &command, 1);
    }

    if (err == ESP_OK)
    {
        command = RV3028_EE_COMMAND_UPDATE;
        err     = rv3028_write_regs(ctx, RV3028_REG_EE_COMMAND, &command, 1);
    }

    if (err == ESP_OK)
    {
        err = rv3028_wait_eeprom_ready(ctx);
    }

    /* Re-enable automatic EEPROM refresh */
    control1 &= ~RV3028_CONTROL1_EERD;
    esp_err_t restore_err = rv3028_write_regs(ctx, RV3028_REG_CONTROL1, &control1, 1);

    return err != ESP_OK ? err : restore_err;
}

static esp_err_t rv3028_wait_eeprom_ready(rv3028_ctx_t *ctx)
{
    esp_err_t err;
    uint8_t   status;

    for (int i = 0; i < RV3028_EEBUSY_TIMEOUT_MS / 10; i++)
    {
        err = rv3028_read_regs(ctx, RV3028_REG_STATUS, &status, 1);
        if (err != ESP_OK)
        {
            return err;
        }

        if ((status & RV3028_STATUS_EEBUSY) == 0)
        {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_ERR_TIMEOUT;
}

static uint8_t rv3028_to_bcd(int value)
{
    return (uint8_t) (((value / 10) << 4) | (value % 10));
}

static int rv3028_from_bcd(uint8_t value)
{
    return ((value >> 4) * 10) + (value & 0x0F);
}

/* Sakamoto's algorithm, returns 0 = Sunday */
static int rv3028_weekday(int year, int month, int day)
{
    static const int offsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

    if (month < 3)
    {
        year -= 1;
    }

    return (year + year / 4 - year / 100 + year / 400 + offsets[month - 1] + day) % 7;
}
