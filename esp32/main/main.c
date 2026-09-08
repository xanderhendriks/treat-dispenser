#include <stdio.h>

#include "ble_remote.h"
#include "ch224a.h"
#include "console.h"
#include "dispenser.h"
#include "drv5055.h"
#include "drv8871.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "max98357a.h"
#include "nvs_flash.h"
#include "rv3028.h"
#include "scheduler.h"
#include "vsense.h"

/* Treat dispenser board: DRV8871 IN1 on GPIO4, IN2 on GPIO5 */
#define MOTOR_IN1_GPIO_NUM 4
#define MOTOR_IN2_GPIO_NUM 5

/*
 * Drum motion. The drum only ever turns one way while dispensing, and the
 * magnets sit close enough together that a hop between two of them is well
 * under a second at the travel speed; the timeout only has to be generous
 * enough to tell a slow start apart from a jam.
 */
#define DISPENSE_DIRECTION  DRV8871_DIRECTION_REVERSE
#define DISPENSE_SPEED_PCT  100
#define DISPENSE_POLL_MS    10
#define DISPENSE_TIMEOUT_MS 5000
#define DISPENSE_BRAKE_MS   150

/*
 * A tada marks arriving at the home position, and the Billy Joel hook plays
 * twice once a treat has been dispensed.
 *
 * The tada doubles as the short acknowledgement: it is what a move backwards
 * plays, having dispensed nothing worth singing about, and what any move driven
 * from the phone plays, the app already showing on screen that it finished.
 */
#define HOME_MELODY        MAX98357A_MELODY_TADA
#define HOME_MELODY_TIMES  1
#define TREAT_MELODY       MAX98357A_MELODY_FOR_THE_LONGEST_TIME
#define TREAT_MELODY_TIMES 2
#define SHORT_MELODY       MAX98357A_MELODY_TADA
#define SHORT_MELODY_TIMES 1
#define MELODY_VOLUME_PCT  75

/*
 * Feeding times. The RV-3028 alarm matches on hour and minute alone, so the
 * scheduler re-arms it for the next of these every time one fires. RTC_INT is
 * on GPIO15, driven open drain by the RV-3028 against a 4k7 pull-up.
 */
#define RTC_INT_GPIO_NUM 15

/* MAX98357A I2S amplifier */
#define AUDIO_BCLK_GPIO_NUM    16
#define AUDIO_LRCLK_GPIO_NUM   17
#define AUDIO_DIN_GPIO_NUM     18
#define AUDIO_SD_MODE_GPIO_NUM 21

/* DRV5055A3 linear Hall sensor: OUT on GPIO1 (ADC1_CH0) through a 1k/10nF RC filter */
#define HALL_OUT_GPIO_NUM 1

/*
 * +9V rail monitor: R5 (100k) from +9V to the tap, R8 (27k) from the tap to
 * GND, C9 (100nF) across R8. GPIO2 is ADC1_CH1. The 27/127 ratio puts a 9V
 * rail at about 1.9V on the pin and saturates the 12 dB input around 14.6V,
 * comfortably above anything D1 lets through.
 */
#define SUPPLY_SENSE_GPIO_NUM      2
#define SUPPLY_SENSE_HIGH_SIDE_R   100000
#define SUPPLY_SENSE_LOW_SIDE_R    27000
#define SUPPLY_UNDERVOLTAGE_MV     7500
#define SUPPLY_OVERVOLTAGE_MV      10500
#define SUPPLY_RANGE_HYSTERESIS_MV 250

/* Shared I2C bus: RV-3028-C7 RTC at 0x52, CH224A USB-PD sink at 0x22 or 0x23 */
#define I2C_SDA_GPIO_NUM 6
#define I2C_SCL_GPIO_NUM 7

/*
 * The CH224A output is the board's only supply rail. D1 (SMBJ12A) clamps that
 * rail and CP1 is a 25V part, so nothing above the 9V that the CFG1 strapping
 * resistor already selects may be requested over I2C.
 */
#define PD_MAX_VOLTAGE CH224A_VOLTAGE_9V

/*
 * Phone link. The ESP32-S3 has no Bluetooth Classic radio, so this is BLE
 * only; WiFi is never brought up and the coexistence arbiter is off, which
 * leaves the whole radio to BLE. New phones may only pair during the first
 * minute after a reset, after which the dispenser talks to bonded phones
 * alone. The console ble_pair command re-opens the window.
 */
#define BLE_DEVICE_NAME       "Treat Dispenser"
#define BLE_PAIRING_WINDOW_MS 60000

static const char *TAG = "main";

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_unit       = NULL;
    i2c_master_bus_handle_t   i2c_bus         = NULL;
    drv8871_handle_t          motor_handle    = NULL;
    max98357a_handle_t        audio_handle    = NULL;
    rv3028_handle_t           rtc_handle      = NULL;
    ch224a_handle_t           pd_handle       = NULL;
    drv5055_handle_t          hall_handle     = NULL;
    vsense_handle_t           supply_handle   = NULL;
    dispenser_handle_t        drum_handle     = NULL;
    scheduler_handle_t        schedule_handle = NULL;
    ble_remote_handle_t       ble_handle      = NULL;

    drv8871_config_t motor_cfg = {
        .in1_gpio_num     = MOTOR_IN1_GPIO_NUM,
        .in2_gpio_num     = MOTOR_IN2_GPIO_NUM,
        .pwm_timer        = LEDC_TIMER_0,
        .in1_pwm_channel  = LEDC_CHANNEL_0,
        .in2_pwm_channel  = LEDC_CHANNEL_1,
        .pwm_frequency_hz = 0,
    };
    max98357a_config_t audio_cfg = {
        .bclk_gpio_num    = AUDIO_BCLK_GPIO_NUM,
        .lrclk_gpio_num   = AUDIO_LRCLK_GPIO_NUM,
        .din_gpio_num     = AUDIO_DIN_GPIO_NUM,
        .sd_mode_gpio_num = AUDIO_SD_MODE_GPIO_NUM,
        .i2s_port         = I2S_NUM_0,
    };
    adc_oneshot_unit_init_cfg_t adc1_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    drv5055_config_t hall_cfg = {
        .out_gpio_num      = HALL_OUT_GPIO_NUM,
        .sensitivity_uv_mt = DRV5055_SENSITIVITY_A3_3V3_UV_MT,
        .supply_mv         = 0,
        .sample_count      = 0,
        .threshold_ut      = 0,
        .hysteresis_ut     = 0,
    };
    vsense_config_t supply_cfg = {
        .in_gpio_num     = SUPPLY_SENSE_GPIO_NUM,
        .high_side_ohms  = SUPPLY_SENSE_HIGH_SIDE_R,
        .low_side_ohms   = SUPPLY_SENSE_LOW_SIDE_R,
        .sample_count    = 0,
        .undervoltage_mv = SUPPLY_UNDERVOLTAGE_MV,
        .overvoltage_mv  = SUPPLY_OVERVOLTAGE_MV,
        .hysteresis_mv   = SUPPLY_RANGE_HYSTERESIS_MV,
    };
    static const scheduler_slot_t dispense_slots[] = {
        {.hour = 7, .minute = 0},
        {.hour = 15, .minute = 0},
        {.hour = 23, .minute = 0},
    };
    scheduler_config_t schedule_cfg = {
        .int_gpio_num    = RTC_INT_GPIO_NUM,
        .slots           = dispense_slots,
        .slot_count      = sizeof(dispense_slots) / sizeof(dispense_slots[0]),
        .task_stack_size = 0,
        .task_priority   = 0,
    };
    dispenser_config_t drum_cfg = {
        .direction        = DISPENSE_DIRECTION,
        .travel_speed_pct = DISPENSE_SPEED_PCT,
        .poll_interval_ms = DISPENSE_POLL_MS,
        .timeout_ms       = DISPENSE_TIMEOUT_MS,
        .brake_ms         = DISPENSE_BRAKE_MS,
        .home_chime =
            {
                .melody       = HOME_MELODY,
                .repeat_count = HOME_MELODY_TIMES,
                .volume_pct   = MELODY_VOLUME_PCT,
            },
        .advance_chime =
            {
                .melody       = TREAT_MELODY,
                .repeat_count = TREAT_MELODY_TIMES,
                .volume_pct   = MELODY_VOLUME_PCT,
            },
        .retreat_chime =
            {
                .melody       = SHORT_MELODY,
                .repeat_count = SHORT_MELODY_TIMES,
                .volume_pct   = MELODY_VOLUME_PCT,
            },
    };
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port                     = -1, /* auto-select */
        .sda_io_num                   = I2C_SDA_GPIO_NUM,
        .scl_io_num                   = I2C_SCL_GPIO_NUM,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    rv3028_config_t rtc_cfg = {
        .i2c_clock_speed_hz = 0,
    };
    ch224a_config_t pd_cfg = {
        .i2c_address        = 0, /* probe 0x22 and then 0x23 */
        .i2c_clock_speed_hz = 0,
        .max_voltage        = PD_MAX_VOLTAGE,
    };
    ble_remote_config_t ble_cfg = {
        .device_name       = BLE_DEVICE_NAME,
        .pairing_window_ms = BLE_PAIRING_WINDOW_MS,
        .remote_chime =
            {
                .melody       = SHORT_MELODY,
                .repeat_count = SHORT_MELODY_TIMES,
                .volume_pct   = MELODY_VOLUME_PCT,
            },
        .task_stack_size = 0,
        .task_priority   = 0,
    };

    ESP_LOGI(TAG, "Treat dispenser %s", esp_app_get_description()->version);

    /* NVS holds the BLE bonds, so a fresh or upgraded partition is worth wiping rather than failing on */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    ESP_ERROR_CHECK(drv8871_init(&motor_cfg, &motor_handle));
    ESP_ERROR_CHECK(max98357a_init(&audio_cfg, &audio_handle));
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_cfg, &adc1_unit));
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    hall_cfg.adc_unit   = adc1_unit;
    supply_cfg.adc_unit = adc1_unit;
    rtc_cfg.bus         = i2c_bus;
    pd_cfg.bus          = i2c_bus;

    ESP_ERROR_CHECK(drv5055_init(&hall_cfg, &hall_handle));
    ESP_ERROR_CHECK(vsense_init(&supply_cfg, &supply_handle));

    drum_cfg.motor_handle = motor_handle;
    drum_cfg.hall_handle  = hall_handle;
    drum_cfg.audio_handle = audio_handle;

    ESP_ERROR_CHECK(dispenser_init(&drum_cfg, &drum_handle));

    esp_err_t err = rv3028_init(&rtc_cfg, &rtc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "RTC unavailable (%s), continuing without it", esp_err_to_name(err));
        rtc_handle = NULL;
    }

    err = ch224a_init(&pd_cfg, &pd_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "USB-PD controller unavailable (%s), continuing without it", esp_err_to_name(err));
        pd_handle = NULL;
    }

    /* The schedule needs the RTC; without it the dispenser is console driven only */
    if (rtc_handle)
    {
        schedule_cfg.rtc_handle  = rtc_handle;
        schedule_cfg.drum_handle = drum_handle;

        err = scheduler_start(&schedule_cfg, &schedule_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Dispensing schedule unavailable (%s)", esp_err_to_name(err));
            schedule_handle = NULL;
        }
    }
    else
    {
        ESP_LOGW(TAG, "No RTC, running without a dispensing schedule");
    }

    ble_cfg.drum_handle     = drum_handle;
    ble_cfg.rtc_handle      = rtc_handle;
    ble_cfg.schedule_handle = schedule_handle;

    err = ble_remote_start(&ble_cfg, &ble_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BLE unavailable (%s), the dispenser is console driven only", esp_err_to_name(err));
        ble_handle = NULL;
    }

    ESP_ERROR_CHECK(console_start(i2c_bus, motor_handle, audio_handle, rtc_handle, pd_handle, hall_handle,
                                  supply_handle, drum_handle, schedule_handle, ble_handle));
}
