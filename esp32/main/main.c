#include <stdio.h>

#include "ble_remote.h"
#include "ch224a.h"
#include "console.h"
#include "dispenser.h"
#include "drv5055.h"
#include "drv8871.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gc9a01a.h"
#include "max98357a.h"
#include "melodies.h"
#include "nvs_flash.h"
#include "rv3028.h"
#include "scheduler.h"
#include "screen.h"
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

/*
 * ER-TFTM1.28-1 round display: a 240x240 GC9A01A on 4-wire SPI. MOSI on GPIO11
 * and SCK on GPIO12 are the ESP32-S3 IOMUX pins for SPI2, so the clock is not
 * held down to what the GPIO matrix can pass; R14 and R15 put 33R in series
 * with both. The backlight is PWM on BLK, which has to keep clear of LEDC
 * timer 0 and channels 0 and 1 that the motor driver already holds.
 *
 * Rotation 90 is what the assembled enclosure needs, the flex tail leaving the
 * glass a quarter turn round from the frame memory's own idea of up.
 */
#define DISPLAY_SPI_HOST          SPI2_HOST
#define DISPLAY_CS_GPIO_NUM       9
#define DISPLAY_DC_GPIO_NUM       10
#define DISPLAY_MOSI_GPIO_NUM     11
#define DISPLAY_SCK_GPIO_NUM      12
#define DISPLAY_RES_GPIO_NUM      13
#define DISPLAY_BLK_GPIO_NUM      14
#define DISPLAY_ROTATION          GC9A01A_ROTATION_90
#define DISPLAY_BACKLIGHT_PCT     80
#define DISPLAY_BACKLIGHT_FADE_MS 400

/*
 * Screen care for a face that shows a live countdown. The countdown redraws
 * every second, so the picture is never static for long and the drawing code
 * deliberately does not report those redraws as activity: that would reset the
 * idle clock every second and the panel would never dim, never nudge and never
 * sleep.
 *
 * Blanking is off. The board has no button, no touch panel and nothing else a
 * person can press, so a blanked panel would stay dark until the next feeding
 * time or a phone connection woke it. Dimming to a third is enough: the panel
 * is bright enough to read dim, and it is black background with thin white
 * strokes, which is the least stressed state the glass has. Turning blanking
 * back on is a supported choice and the screen copes with it, the display then
 * lighting up at each feeding time and going dark ten minutes later.
 */
#define DISPLAY_DIM_AFTER_MS   300000
#define DISPLAY_DIM_PCT        30
#define DISPLAY_BLANK_AFTER_MS GC9A01A_CARE_NEVER

/*
 * The clock face does not stay in one place. Once a minute it steps to the next
 * of eight positions round a ring twelve pixels out from the middle, which is
 * far enough that a segment lands clear of where it was: a step has to be wider
 * than the stroke to be worth anything, and the strokes here are five pixels.
 * Each position is then occupied an eighth of the time, and one full circuit
 * takes eight minutes.
 *
 * Twelve pixels because of how it looks rather than because of what fits: the
 * three row layout would allow nineteen, but a nineteen pixel hop once a minute
 * reads as the face jumping rather than drifting, and twelve already steps eight
 * and a half pixels at a time against a five pixel stroke. The screen clamps
 * this to whatever the layout actually affords, so growing something on the face
 * costs travel and says so rather than pushing anything under the bezel.
 */
#define DISPLAY_DRIFT_INTERVAL_MS 60000
#define DISPLAY_DRIFT_RADIUS_PX   12

/*
 * Start-up sequence. The dog is on the glass from the first frame and stays
 * there while the drum goes looking for itself, which is the one part of the
 * boot that takes a visible moment; a tada says it found itself, and the face
 * arrives a couple of seconds after that. Holding him rather than timing him
 * out is why the screen takes SCREEN_CELEBRATE_HOLD: nobody knows in advance how
 * long the drum will take, and a portrait that vanished mid-move would look like
 * a fault rather than a greeting.
 */
#define DISPLAY_SPLASH_LINGER_MS 2000

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

static void main_on_drum_move(bool moving, void *ctx);

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
    gc9a01a_handle_t          display_handle  = NULL;
    screen_handle_t           screen_handle   = NULL;

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
    gc9a01a_config_t display_cfg = {
        .spi_host                   = DISPLAY_SPI_HOST,
        .sck_gpio_num               = DISPLAY_SCK_GPIO_NUM,
        .mosi_gpio_num              = DISPLAY_MOSI_GPIO_NUM,
        .cs_gpio_num                = DISPLAY_CS_GPIO_NUM,
        .dc_gpio_num                = DISPLAY_DC_GPIO_NUM,
        .rst_gpio_num               = DISPLAY_RES_GPIO_NUM,
        .backlight_gpio_num         = DISPLAY_BLK_GPIO_NUM,
        .spi_clock_speed_hz         = 0,
        .rotation                   = DISPLAY_ROTATION,
        .dot_inversion              = GC9A01A_DOT_INVERSION_4_DOT,
        .backlight_pwm_timer        = LEDC_TIMER_1,
        .backlight_pwm_channel      = LEDC_CHANNEL_2,
        .backlight_pwm_frequency_hz = 0,
        .transfer_chunk_bytes       = 0,
    };
    gc9a01a_care_config_t display_care_cfg = {
        .nudge_after_ms     = 0, /* the datasheet's own advice: shift a long lived border every minute */
        .dim_after_ms       = DISPLAY_DIM_AFTER_MS,
        .blank_after_ms     = DISPLAY_BLANK_AFTER_MS,
        .warn_after_ms      = 0,
        .dim_backlight_pct  = DISPLAY_DIM_PCT,
        .nudge_amplitude_px = 0,
        .task_stack_size    = 0,
        .task_priority      = 0,
    };
    screen_config_t screen_cfg = {
        .drift_interval_ms = DISPLAY_DRIFT_INTERVAL_MS,
        .drift_radius_px   = DISPLAY_DRIFT_RADIUS_PX,
        .splash            = true,
        .task_stack_size   = 0,
        .task_priority     = 0,
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

    /*
     * The display is decoration: a board that cannot bring it up still feeds
     * the dog, so a failure here is logged and stepped over. The panel comes up
     * with its frame memory cleared and the backlight off, and stays that way
     * until the clock face has drawn its first frame further down, which saves
     * showing anybody a lit black circle while the rest of the board starts.
     */
    esp_err_t display_err = gc9a01a_init(&display_cfg, &display_handle);
    if (display_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Display unavailable (%s), continuing without it", esp_err_to_name(display_err));
        display_handle = NULL;
    }
    else
    {
        display_err = gc9a01a_care_start(display_handle, &display_care_cfg);
        if (display_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Screen care unavailable (%s), nothing is watching for image sticking",
                     esp_err_to_name(display_err));
        }
    }

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

    /*
     * The clock face needs the RTC for the time, the schedule for what it is
     * counting down to and the radio for the Bluetooth icon, so it goes up after
     * all three. It draws its first frame before returning, which is why the
     * backlight only comes on afterwards.
     */
    if (display_handle)
    {
        screen_cfg.display_handle  = display_handle;
        screen_cfg.rtc_handle      = rtc_handle;
        screen_cfg.schedule_handle = schedule_handle;
        screen_cfg.ble_handle      = ble_handle;
        screen_cfg.audio_handle    = audio_handle;

        err = screen_start(&screen_cfg, &screen_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Clock face unavailable (%s), the display stays blank", esp_err_to_name(err));
            screen_handle = NULL;
        }

        gc9a01a_fade_backlight(display_handle, DISPLAY_BACKLIGHT_PCT, DISPLAY_BACKLIGHT_FADE_MS);
    }

    ESP_ERROR_CHECK(console_start(i2c_bus, motor_handle, audio_handle, rtc_handle, pd_handle, hall_handle,
                                  supply_handle, drum_handle, schedule_handle, ble_handle, display_handle,
                                  screen_handle));

    /*
     * Find out where the drum is standing before anything asks. Left until now
     * so that the console and the phone are already up: the nudge takes a
     * moment, and the app showing "position not known" turning into a slot
     * number is friendlier than a board that says nothing while it moves.
     *
     * A drum that has never been calibrated has no slots to tell apart, which is
     * not an error worth stopping the boot for; it just leaves the position
     * unknown until drum_calibrate has been run.
     */
    dispenser_result_t start_position;

    err = dispenser_find_position(drum_handle, &start_position);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Drum is on slot %d%s", start_position.slot,
                 start_position.already_there ? ", already known" : "");

        /*
         * dispenser_find_position() stays deliberately quiet, being a move
         * nobody asked for, so the tada that says the dispenser is up and knows
         * where it is belongs here. Only on success: a triumphant noise over a
         * drum that has no idea where it is would be a lie.
         */
        max98357a_play(audio_handle, HOME_MELODY, HOME_MELODY_TIMES, MELODY_VOLUME_PCT);
    }
    else if (err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "No slot map, so the drum position stays unknown until it is calibrated");
    }
    else
    {
        ESP_LOGE(TAG, "Could not work out which slot the drum is on (%s)", esp_err_to_name(err));
    }

    /* Playback runs in the background, so the tada carries on over this wait rather than delaying it */
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_SPLASH_LINGER_MS));
    screen_celebrate(screen_handle, 0);

    /*
     * From here on the dog goes up whenever the drum turns, wherever the order
     * came from. Hooked up only now, after the start-up sequence has finished
     * with him: the move above would otherwise have ended the splash early,
     * finding a drum that had stopped and no chime yet to wait on.
     */
    if (screen_handle)
    {
        dispenser_set_move_observer(drum_handle, main_on_drum_move, screen_handle);
    }
}

/*
 * Hold him while the drum turns, then until the chime it ends on has finished.
 * Runs in whichever task asked for the move: the schedule, the phone or the
 * console.
 */
static void main_on_drum_move(bool moving, void *ctx)
{
    screen_celebrate((screen_handle_t) ctx, moving ? SCREEN_CELEBRATE_HOLD : SCREEN_CELEBRATE_UNTIL_QUIET);
}
