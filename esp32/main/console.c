#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "melodies.h"
#include "sdkconfig.h"

/*
 * Moves driven from the console are acknowledged the same way as those driven
 * from the phone: a short tada rather than the full treat tune, the printed
 * result already saying the move finished.
 */
static const dispenser_chime_t s_manual_chime = {
    .melody       = SHORT_MELODY,
    .repeat_count = SHORT_MELODY_TIMES,
    .volume_pct   = MELODY_VOLUME_PCT,
};

static const char             *TAG = "console";
static i2c_master_bus_handle_t s_i2c_bus;
static drv8871_handle_t        s_motor_handle;
static max98357a_handle_t      s_audio_handle;
static rv3028_handle_t         s_rtc_handle;
static ch224a_handle_t         s_pd_handle;
static drv5055_handle_t        s_hall_handle;
static vsense_handle_t         s_supply_handle;
static dispenser_handle_t      s_drum_handle;
static scheduler_handle_t      s_schedule_handle;
static ble_remote_handle_t     s_ble_handle;
static esp_console_repl_t     *s_repl;

/* i2c_scan sweeps the 7-bit addresses that are not reserved by the standard */
#define I2C_SCAN_FIRST_ADDRESS 0x08
#define I2C_SCAN_LAST_ADDRESS  0x77
#define I2C_SCAN_TIMEOUT_MS    50

/* hall_watch polls fast enough to catch a magnet sweeping past, and reports at a readable rate */
#define HALL_WATCH_POLL_MS     50
#define HALL_WATCH_REPORT_MS   250
#define HALL_WATCH_MIN_SECONDS 1
#define HALL_WATCH_MAX_SECONDS 60

typedef struct speed_args
{
    struct arg_int *value;
    struct arg_end *end;
} speed_args_t;

typedef struct direction_args
{
    struct arg_int *value;
    struct arg_end *end;
} direction_args_t;

typedef struct play_args
{
    struct arg_int *melody;
    struct arg_int *times;
    struct arg_int *volume;
    struct arg_end *end;
} play_args_t;

typedef struct time_set_args
{
    struct arg_str *date;
    struct arg_str *time;
    struct arg_end *end;
} time_set_args_t;

typedef struct pd_read_args
{
    struct arg_int *reg;
    struct arg_end *end;
} pd_read_args_t;

typedef struct pd_voltage_args
{
    struct arg_int *gear;
    struct arg_end *end;
} pd_voltage_args_t;

typedef struct hall_watch_args
{
    struct arg_int *seconds;
    struct arg_end *end;
} hall_watch_args_t;

typedef struct hall_threshold_args
{
    struct arg_int *threshold;
    struct arg_int *hysteresis;
    struct arg_end *end;
} hall_threshold_args_t;

typedef struct supply_calibrate_args
{
    struct arg_int *millivolts;
    struct arg_end *end;
} supply_calibrate_args_t;

typedef struct supply_range_args
{
    struct arg_int *undervoltage;
    struct arg_int *overvoltage;
    struct arg_int *hysteresis;
    struct arg_end *end;
} supply_range_args_t;

typedef struct dispense_speed_args
{
    struct arg_int *value;
    struct arg_end *end;
} dispense_speed_args_t;

typedef struct slot_args
{
    struct arg_int *number;
    struct arg_end *end;
} slot_args_t;

typedef struct drum_calibrate_args
{
    struct arg_int *revolutions;
    struct arg_int *speed;
    struct arg_int *gate;
    struct arg_lit *forward;
    struct arg_lit *home_weak;
    struct arg_lit *zero;
    struct arg_lit *dry_run;
    struct arg_end *end;
} drum_calibrate_args_t;

static speed_args_t          s_speed_args;
static direction_args_t      s_direction_args;
static play_args_t           s_play_args;
static time_set_args_t       s_time_set_args;
static pd_read_args_t        s_pd_read_args;
static pd_voltage_args_t     s_pd_voltage_args;
static hall_watch_args_t     s_hall_watch_args;
static hall_threshold_args_t s_hall_threshold_args;

static supply_calibrate_args_t s_supply_calibrate_args;
static supply_range_args_t     s_supply_range_args;
static dispense_speed_args_t   s_dispense_speed_args;
static drum_calibrate_args_t   s_drum_calibrate_args;
static slot_args_t             s_slot_args;

static esp_err_t register_speed_command(void);
static esp_err_t register_direction_command(void);
static esp_err_t register_stop_command(void);
static esp_err_t register_brake_command(void);
static esp_err_t register_status_command(void);
static esp_err_t register_play_command(void);
static esp_err_t register_quiet_command(void);
static esp_err_t register_time_set_command(void);
static esp_err_t register_time_get_command(void);
static esp_err_t register_i2c_scan_command(void);
static esp_err_t register_pd_status_command(void);
static esp_err_t register_pd_dump_command(void);
static esp_err_t register_pd_read_command(void);
static esp_err_t register_pd_voltage_command(void);
static esp_err_t register_hall_command(void);
static esp_err_t register_hall_zero_command(void);
static esp_err_t register_hall_watch_command(void);
static esp_err_t register_hall_threshold_command(void);
static esp_err_t register_supply_command(void);
static esp_err_t register_supply_calibrate_command(void);
static esp_err_t register_supply_range_command(void);
static esp_err_t register_home_command(void);
static esp_err_t register_next_command(void);
static esp_err_t register_prev_command(void);
static esp_err_t register_dispense_speed_command(void);
static esp_err_t register_schedule_command(void);
static esp_err_t register_ble_command(void);
static esp_err_t register_ble_pair_command(void);
static esp_err_t register_ble_forget_command(void);
static esp_err_t register_drum_calibrate_command(void);
static esp_err_t register_drum_map_command(void);
static esp_err_t register_drum_slot_command(void);
static esp_err_t register_drum_forget_command(void);
static esp_err_t register_slot_command(void);
static esp_err_t register_locate_command(void);

static int cmd_speed(int argc, char **argv);
static int cmd_direction(int argc, char **argv);
static int cmd_stop(int argc, char **argv);
static int cmd_brake(int argc, char **argv);
static int cmd_status(int argc, char **argv);
static int cmd_play(int argc, char **argv);
static int cmd_quiet(int argc, char **argv);
static int cmd_time_set(int argc, char **argv);
static int cmd_time_get(int argc, char **argv);
static int cmd_i2c_scan(int argc, char **argv);
static int cmd_pd_status(int argc, char **argv);
static int cmd_pd_dump(int argc, char **argv);
static int cmd_pd_read(int argc, char **argv);
static int cmd_pd_voltage(int argc, char **argv);
static int cmd_hall(int argc, char **argv);
static int cmd_hall_zero(int argc, char **argv);
static int cmd_hall_watch(int argc, char **argv);
static int cmd_hall_threshold(int argc, char **argv);
static int cmd_supply(int argc, char **argv);
static int cmd_supply_calibrate(int argc, char **argv);
static int cmd_supply_range(int argc, char **argv);
static int cmd_home(int argc, char **argv);
static int cmd_next(int argc, char **argv);
static int cmd_prev(int argc, char **argv);
static int cmd_dispense_speed(int argc, char **argv);
static int cmd_schedule(int argc, char **argv);
static int cmd_ble(int argc, char **argv);
static int cmd_ble_pair(int argc, char **argv);
static int cmd_ble_forget(int argc, char **argv);
static int cmd_drum_calibrate(int argc, char **argv);
static int cmd_drum_map(int argc, char **argv);
static int cmd_drum_slot(int argc, char **argv);
static int cmd_drum_forget(int argc, char **argv);
static int cmd_slot(int argc, char **argv);
static int cmd_locate(int argc, char **argv);

static void print_hall_reading(const drv5055_reading_t *reading);
static void print_supply_reading(const vsense_reading_t *reading);
static void print_move_result(const dispenser_result_t *result);
static void print_field_mt(int32_t field_ut);
static int  report_move_error(const char *what, esp_err_t err);
static void print_calibration(const dispenser_cal_result_t *result);

esp_err_t console_start(i2c_master_bus_handle_t i2c_bus, drv8871_handle_t motor_handle, max98357a_handle_t audio_handle,
                        rv3028_handle_t rtc_handle, ch224a_handle_t pd_handle, drv5055_handle_t hall_handle,
                        vsense_handle_t supply_handle, dispenser_handle_t drum_handle,
                        scheduler_handle_t schedule_handle, ble_remote_handle_t ble_handle)
{
    esp_err_t err;

    /* rtc_handle, pd_handle, schedule_handle and ble_handle may be NULL when that part is unavailable */
    if (i2c_bus == NULL || motor_handle == NULL || audio_handle == NULL || hall_handle == NULL ||
        supply_handle == NULL || drum_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_bus         = i2c_bus;
    s_motor_handle    = motor_handle;
    s_audio_handle    = audio_handle;
    s_rtc_handle      = rtc_handle;
    s_pd_handle       = pd_handle;
    s_hall_handle     = hall_handle;
    s_supply_handle   = supply_handle;
    s_drum_handle     = drum_handle;
    s_schedule_handle = schedule_handle;
    s_ble_handle      = ble_handle;

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt                    = "treat_dispenser>";
    repl_config.max_cmdline_length        = 64;

    esp_console_dev_uart_config_t repl_uart = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    err = esp_console_new_repl_uart(&repl_uart, &repl_config, &s_repl);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create UART REPL (%s)", esp_err_to_name(err));
        return err;
    }

    esp_console_register_help_command();

    err = register_speed_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register speed command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_direction_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register direction command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_stop_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register stop command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_brake_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register brake command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_status_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register status command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_play_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register play command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_quiet_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register quiet command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_time_set_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register time_set command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_time_get_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register time_get command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_i2c_scan_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register i2c_scan command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_pd_status_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register pd_status command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_pd_dump_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register pd_dump command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_pd_read_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register pd_read command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_pd_voltage_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register pd_voltage command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_hall_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register hall command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_hall_zero_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register hall_zero command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_hall_watch_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register hall_watch command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_hall_threshold_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register hall_threshold command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_supply_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register supply command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_supply_calibrate_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register supply_calibrate command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_supply_range_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register supply_range command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_home_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register home command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_next_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register next command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_prev_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register prev command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_dispense_speed_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register dispense_speed command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_schedule_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register schedule command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_ble_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register ble command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_ble_pair_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register ble_pair command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_ble_forget_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register ble_forget command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_drum_calibrate_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register drum_calibrate command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_drum_map_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register drum_map command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_drum_slot_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register drum_slot command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_drum_forget_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register drum_forget command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_slot_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register slot command (%s)", esp_err_to_name(err));
        return err;
    }

    err = register_locate_command();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register locate command (%s)", esp_err_to_name(err));
        return err;
    }

    err = esp_console_start_repl(s_repl);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start REPL (%s)", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t register_speed_command(void)
{
    s_speed_args.value = arg_int1(NULL, NULL, "<0-100>", "Motor speed in percent");
    s_speed_args.end   = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "speed",
        .help     = "Set the motor speed in percent (0 lets the motor coast)",
        .hint     = NULL,
        .func     = &cmd_speed,
        .argtable = &s_speed_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_direction_command(void)
{
    s_direction_args.value = arg_int1(NULL, NULL, "<0|1>", "Motor direction: 0 = forward, 1 = reverse");
    s_direction_args.end   = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "direction",
        .help     = "Set the motor rotation direction",
        .hint     = NULL,
        .func     = &cmd_direction,
        .argtable = &s_direction_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_stop_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "stop",
        .help    = "Stop the motor (coast)",
        .hint    = NULL,
        .func    = &cmd_stop,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_brake_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "brake",
        .help    = "Actively brake the motor",
        .hint    = NULL,
        .func    = &cmd_brake,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_status_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "status",
        .help    = "Show the current motor speed and direction",
        .hint    = NULL,
        .func    = &cmd_status,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_play_command(void)
{
    s_play_args.melody = arg_int1(NULL, NULL, "<melody>", "Melody index (0 = For the Longest Time)");
    s_play_args.times  = arg_int1(NULL, NULL, "<times>", "Number of times to play the melody");
    s_play_args.volume = arg_int1(NULL, NULL, "<0-100>", "Volume in percent");
    s_play_args.end    = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command  = "play",
        .help     = "Play a melody a number of times at the given volume",
        .hint     = NULL,
        .func     = &cmd_play,
        .argtable = &s_play_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_quiet_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "quiet",
        .help    = "Stop melody playback",
        .hint    = NULL,
        .func    = &cmd_quiet,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_time_set_command(void)
{
    s_time_set_args.date = arg_str1(NULL, NULL, "<YYYY-MM-DD>", "Date to set");
    s_time_set_args.time = arg_str1(NULL, NULL, "<HH:MM:SS>", "Time to set (24h)");
    s_time_set_args.end  = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command  = "time_set",
        .help     = "Set the RTC date and time",
        .hint     = NULL,
        .func     = &cmd_time_set,
        .argtable = &s_time_set_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_time_get_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "time_get",
        .help    = "Show the RTC date and time",
        .hint    = NULL,
        .func    = &cmd_time_get,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_i2c_scan_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "i2c_scan",
        .help    = "Probe every 7-bit address on the I2C bus",
        .hint    = NULL,
        .func    = &cmd_i2c_scan,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_pd_status_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "pd_status",
        .help    = "Show the CH224A protocol status and available current",
        .hint    = NULL,
        .func    = &cmd_pd_status,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_pd_dump_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "pd_dump",
        .help    = "Read every documented CH224A register",
        .hint    = NULL,
        .func    = &cmd_pd_dump,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_pd_read_command(void)
{
    s_pd_read_args.reg = arg_int1(NULL, NULL, "<reg>", "Register address (decimal, or 0x for hex)");
    s_pd_read_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "pd_read",
        .help     = "Read a single CH224A register",
        .hint     = NULL,
        .func     = &cmd_pd_read,
        .argtable = &s_pd_read_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_pd_voltage_command(void)
{
    s_pd_voltage_args.gear =
        arg_int1(NULL, NULL, "<gear>", "Voltage gear: 0 = 5V, 1 = 9V, 2 = 12V, 3 = 15V, 4 = 20V, 5 = 28V");
    s_pd_voltage_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "pd_voltage",
        .help     = "Request a USB-PD voltage gear (limited to what the board tolerates)",
        .hint     = NULL,
        .func     = &cmd_pd_voltage,
        .argtable = &s_pd_voltage_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_hall_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "hall",
        .help    = "Read the Hall sensor field, in millitesla",
        .hint    = NULL,
        .func    = &cmd_hall,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_hall_zero_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "hall_zero",
        .help    = "Capture the current Hall reading as the zero-field reference (keep magnets away)",
        .hint    = NULL,
        .func    = &cmd_hall_zero,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_hall_watch_command(void)
{
    s_hall_watch_args.seconds = arg_int1(NULL, NULL, "<seconds>", "How long to watch, 1-60");
    s_hall_watch_args.end     = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "hall_watch",
        .help     = "Stream Hall readings, for aligning the magnet on the dispenser wheel",
        .hint     = NULL,
        .func     = &cmd_hall_watch,
        .argtable = &s_hall_watch_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_hall_threshold_command(void)
{
    s_hall_threshold_args.threshold  = arg_int0(NULL, NULL, "<uT>", "Detection threshold in microtesla");
    s_hall_threshold_args.hysteresis = arg_int0(NULL, NULL, "<uT>", "Hysteresis in microtesla");
    s_hall_threshold_args.end        = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command  = "hall_threshold",
        .help     = "Show or set the magnet detection threshold and hysteresis",
        .hint     = NULL,
        .func     = &cmd_hall_threshold,
        .argtable = &s_hall_threshold_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_supply_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "supply",
        .help    = "Show the +9V rail voltage",
        .hint    = NULL,
        .func    = &cmd_supply,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_supply_calibrate_command(void)
{
    s_supply_calibrate_args.millivolts = arg_int1(NULL, NULL, "<mV>", "Rail voltage as measured with a meter");
    s_supply_calibrate_args.end        = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "supply_calibrate",
        .help     = "Trim the rail voltage gain against an external measurement",
        .hint     = NULL,
        .func     = &cmd_supply_calibrate,
        .argtable = &s_supply_calibrate_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_supply_range_command(void)
{
    s_supply_range_args.undervoltage = arg_int0(NULL, NULL, "<under mV>", "Lower limit, 0 disables it");
    s_supply_range_args.overvoltage  = arg_int0(NULL, NULL, "<over mV>", "Upper limit, 0 disables it");
    s_supply_range_args.hysteresis   = arg_int0(NULL, NULL, "<hyst mV>", "Hysteresis around both limits");
    s_supply_range_args.end          = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command  = "supply_range",
        .help     = "Show or set the acceptable rail voltage window",
        .hint     = NULL,
        .func     = &cmd_supply_range,
        .argtable = &s_supply_range_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_home_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "home",
        .help    = "Turn the drum until it parks on the home magnet (the reversed one, reading a negative field)",
        .hint    = NULL,
        .func    = &cmd_home,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_next_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "next",
        .help    = "Turn the drum on to the next magnet, whatever its polarity",
        .hint    = NULL,
        .func    = &cmd_next,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_prev_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "prev",
        .help    = "Turn the drum back to the previous magnet, dispensing nothing",
        .hint    = NULL,
        .func    = &cmd_prev,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_dispense_speed_command(void)
{
    s_dispense_speed_args.value = arg_int0(NULL, NULL, "<1-100>", "Motor speed used by home and next, in percent");
    s_dispense_speed_args.end   = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "dispense_speed",
        .help     = "Show or set the motor speed the home and next commands drive at",
        .hint     = NULL,
        .func     = &cmd_dispense_speed,
        .argtable = &s_dispense_speed_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_schedule_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "schedule",
        .help    = "Show the automatic dispensing times and the one the RTC alarm is armed for",
        .hint    = NULL,
        .func    = &cmd_schedule,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_ble_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "ble",
        .help    = "Show the state of the phone link and the pairing window",
        .hint    = NULL,
        .func    = &cmd_ble,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_ble_pair_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "ble_pair",
        .help    = "Re-open the pairing window so another phone can bond without a reset",
        .hint    = NULL,
        .func    = &cmd_ble_pair,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_ble_forget_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "ble_forget",
        .help    = "Delete every stored bond, so paired phones have to forget the dispenser and pair again",
        .hint    = NULL,
        .func    = &cmd_ble_forget,
    };

    return esp_console_cmd_register(&cmd);
}

static int cmd_speed(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &s_speed_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_speed_args.end, argv[0]);
        return 1;
    }

    int speed = s_speed_args.value->ival[0];
    if (speed < 0 || speed > 100)
    {
        printf("Speed must be between 0 and 100\n");
        return 1;
    }

    esp_err_t err = drv8871_set_speed(s_motor_handle, (uint32_t) speed);
    if (err != ESP_OK)
    {
        printf("Failed to set speed (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Speed set to %d%%\n", speed);
    return 0;
}

static int cmd_direction(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &s_direction_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_direction_args.end, argv[0]);
        return 1;
    }

    int direction = s_direction_args.value->ival[0];
    if (direction != 0 && direction != 1)
    {
        printf("Direction must be 0 (forward) or 1 (reverse)\n");
        return 1;
    }

    esp_err_t err =
        drv8871_set_direction(s_motor_handle, direction ? DRV8871_DIRECTION_REVERSE : DRV8871_DIRECTION_FORWARD);
    if (err != ESP_OK)
    {
        printf("Failed to set direction (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Direction set to %s\n", direction ? "reverse" : "forward");
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    esp_err_t err = drv8871_coast(s_motor_handle);
    if (err != ESP_OK)
    {
        printf("Failed to stop motor (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Motor stopped (coasting)\n");
    return 0;
}

static int cmd_brake(int argc, char **argv)
{
    esp_err_t err = drv8871_brake(s_motor_handle);
    if (err != ESP_OK)
    {
        printf("Failed to brake motor (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Motor braking\n");
    return 0;
}

static int cmd_play(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &s_play_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_play_args.end, argv[0]);
        return 1;
    }

    int melody = s_play_args.melody->ival[0];
    int times  = s_play_args.times->ival[0];
    int volume = s_play_args.volume->ival[0];

    if (melody < 0 || melody >= MAX98357A_MELODY_COUNT)
    {
        printf("Melody must be between 0 and %d\n", MAX98357A_MELODY_COUNT - 1);
        return 1;
    }

    if (times < 1)
    {
        printf("Times must be 1 or more\n");
        return 1;
    }

    if (volume < 0 || volume > 100)
    {
        printf("Volume must be between 0 and 100\n");
        return 1;
    }

    esp_err_t err = max98357a_play(s_audio_handle, (max98357a_melody_t) melody, (uint32_t) times, (uint32_t) volume);
    if (err != ESP_OK)
    {
        printf("Failed to play melody (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Playing melody %d, %d time(s) at %d%% volume\n", melody, times, volume);
    return 0;
}

static int cmd_quiet(int argc, char **argv)
{
    esp_err_t err = max98357a_stop(s_audio_handle);
    if (err != ESP_OK)
    {
        printf("Failed to stop playback (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Playback stopped\n");
    return 0;
}

static int cmd_time_set(int argc, char **argv)
{
    if (s_rtc_handle == NULL)
    {
        printf("RTC not available\n");
        return 1;
    }

    int nerrors = arg_parse(argc, argv, (void **) &s_time_set_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_time_set_args.end, argv[0]);
        return 1;
    }

    int year, month, day, hour, minute, second;

    if (sscanf(s_time_set_args.date->sval[0], "%4d-%2d-%2d", &year, &month, &day) != 3 ||
        sscanf(s_time_set_args.time->sval[0], "%2d:%2d:%2d", &hour, &minute, &second) != 3)
    {
        printf("Expected date as YYYY-MM-DD and time as HH:MM:SS\n");
        return 1;
    }

    struct tm time = {
        .tm_year = year - 1900,
        .tm_mon  = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min  = minute,
        .tm_sec  = second,
    };

    esp_err_t err = rv3028_set_time(s_rtc_handle, &time);
    if (err != ESP_OK)
    {
        printf("Failed to set time (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Time set to %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    /* The armed alarm was picked against the old time, so it has to be recomputed */
    if (s_schedule_handle)
    {
        err = scheduler_reschedule(s_schedule_handle);
        if (err != ESP_OK)
        {
            printf("Failed to update the dispensing schedule (%s)\n", esp_err_to_name(err));
            return 1;
        }
    }

    return 0;
}

static int cmd_time_get(int argc, char **argv)
{
    static const char *weekdays[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

    struct tm time;
    bool      valid = false;

    if (s_rtc_handle == NULL)
    {
        printf("RTC not available\n");
        return 1;
    }

    esp_err_t err = rv3028_get_time(s_rtc_handle, &time);
    if (err != ESP_OK)
    {
        printf("Failed to get time (%s)\n", esp_err_to_name(err));
        return 1;
    }

    err = rv3028_is_time_valid(s_rtc_handle, &valid);
    if (err != ESP_OK)
    {
        printf("Failed to get time validity (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("%s %04d-%02d-%02d %02d:%02d:%02d%s\n", weekdays[time.tm_wday % 7], time.tm_year + 1900, time.tm_mon + 1,
           time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, valid ? "" : " (time not set since power loss)");
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    uint32_t            speed;
    drv8871_direction_t direction;

    esp_err_t err = drv8871_get_speed(s_motor_handle, &speed);
    if (err != ESP_OK)
    {
        printf("Failed to get speed (%s)\n", esp_err_to_name(err));
        return 1;
    }

    err = drv8871_get_direction(s_motor_handle, &direction);
    if (err != ESP_OK)
    {
        printf("Failed to get direction (%s)\n", esp_err_to_name(err));
        return 1;
    }

    bool playing = false;
    err          = max98357a_is_playing(s_audio_handle, &playing);
    if (err != ESP_OK)
    {
        printf("Failed to get playback state (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Speed: %lu%%, direction: %s, audio: %s\n", (unsigned long) speed,
           direction == DRV8871_DIRECTION_REVERSE ? "reverse" : "forward", playing ? "playing" : "idle");
    return 0;
}

static int cmd_i2c_scan(int argc, char **argv)
{
    int found = 0;

    for (uint8_t address = I2C_SCAN_FIRST_ADDRESS; address <= I2C_SCAN_LAST_ADDRESS; address++)
    {
        if (i2c_master_probe(s_i2c_bus, address, I2C_SCAN_TIMEOUT_MS) == ESP_OK)
        {
            printf("Device at 0x%02X\n", address);
            found++;
        }
    }

    if (found == 0)
    {
        printf("No devices found, check wiring, pull-ups and power\n");
        return 1;
    }

    printf("%d device(s) found\n", found);
    return 0;
}

static int cmd_pd_status(int argc, char **argv)
{
    uint8_t  address;
    uint8_t  status;
    uint16_t current_ma;

    if (s_pd_handle == NULL)
    {
        printf("USB-PD controller not available\n");
        return 1;
    }

    esp_err_t err = ch224a_get_status(s_pd_handle, &status);
    if (err != ESP_OK)
    {
        printf("Failed to read status (%s)\n", esp_err_to_name(err));
        return 1;
    }

    err = ch224a_get_max_current_ma(s_pd_handle, &current_ma);
    if (err != ESP_OK)
    {
        printf("Failed to read available current (%s)\n", esp_err_to_name(err));
        return 1;
    }

    ch224a_get_address(s_pd_handle, &address);

    printf("CH224A at 0x%02X, status 0x%02X\n", address, status);
    printf("  BC:  %s\n", status & CH224A_STATUS_BC ? "yes" : "no");
    printf("  QC2: %s\n", status & CH224A_STATUS_QC2 ? "yes" : "no");
    printf("  QC3: %s\n", status & CH224A_STATUS_QC3 ? "yes" : "no");
    printf("  PD:  %s\n", status & CH224A_STATUS_PD ? "yes" : "no");
    printf("  EPR: %s%s\n", status & CH224A_STATUS_EPR_ACTIVE ? "active" : "inactive",
           status & CH224A_STATUS_EPR_EXISTS ? ", offered by supply" : "");
    printf("  AVS: %s\n", status & CH224A_STATUS_AVS_EXISTS ? "offered by supply" : "not offered");

    /* The current register only carries a PD negotiated value */
    if (status & CH224A_STATUS_PD)
    {
        printf("Available current: %u mA\n", (unsigned) current_ma);
    }
    else
    {
        printf("Available current: unknown without a PD handshake (register reads 0x%02X)\n",
               (unsigned) (current_ma / CH224A_CURRENT_STEP_MA));
    }

    ch224a_voltage_t requested;
    if (ch224a_get_requested_voltage(s_pd_handle, &requested) == ESP_OK)
    {
        printf("Last request: %lu mV\n", (unsigned long) ch224a_voltage_to_mv(requested));
    }
    else
    {
        printf("Last request: none, the chip runs on the CFG1 strapping resistor gear\n");
    }

    return 0;
}

static int cmd_pd_dump(int argc, char **argv)
{
    static ch224a_register_t registers[CH224A_REGISTER_COUNT];

    size_t count = 0;

    if (s_pd_handle == NULL)
    {
        printf("USB-PD controller not available\n");
        return 1;
    }

    esp_err_t err = ch224a_read_all_registers(s_pd_handle, registers, CH224A_REGISTER_COUNT, &count);
    if (err != ESP_OK)
    {
        printf("Failed to dump registers (%s)\n", esp_err_to_name(err));
        return 1;
    }

    for (size_t i = 0; i < count; i++)
    {
        if (registers[i].err == ESP_OK)
        {
            printf("0x%02X  0x%02X  %s\n", registers[i].address, registers[i].value,
                   ch224a_register_name(registers[i].address));
        }
        else
        {
            printf("0x%02X  --    %s (%s)\n", registers[i].address, ch224a_register_name(registers[i].address),
                   esp_err_to_name(registers[i].err));
        }
    }

    return 0;
}

static int cmd_pd_read(int argc, char **argv)
{
    uint8_t value;

    if (s_pd_handle == NULL)
    {
        printf("USB-PD controller not available\n");
        return 1;
    }

    int nerrors = arg_parse(argc, argv, (void **) &s_pd_read_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_pd_read_args.end, argv[0]);
        return 1;
    }

    int reg = s_pd_read_args.reg->ival[0];
    if (reg < 0 || reg > 0xFF)
    {
        printf("Register address must be between 0 and 255\n");
        return 1;
    }

    esp_err_t err = ch224a_read_register(s_pd_handle, (uint8_t) reg, &value);
    if (err != ESP_OK)
    {
        printf("Failed to read register 0x%02X (%s)\n", reg, esp_err_to_name(err));
        return 1;
    }

    printf("0x%02X  0x%02X  %s\n", reg, value, ch224a_register_name((uint8_t) reg));
    return 0;
}

static int cmd_pd_voltage(int argc, char **argv)
{
    if (s_pd_handle == NULL)
    {
        printf("USB-PD controller not available\n");
        return 1;
    }

    int nerrors = arg_parse(argc, argv, (void **) &s_pd_voltage_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_pd_voltage_args.end, argv[0]);
        return 1;
    }

    int gear = s_pd_voltage_args.gear->ival[0];
    if (gear < CH224A_VOLTAGE_5V || gear > CH224A_VOLTAGE_28V)
    {
        printf("Gear must be between %d and %d\n", CH224A_VOLTAGE_5V, CH224A_VOLTAGE_28V);
        return 1;
    }

    esp_err_t err = ch224a_set_voltage(s_pd_handle, (ch224a_voltage_t) gear);
    if (err == ESP_ERR_INVALID_ARG)
    {
        printf("Gear %d (%lu mV) is above what this board tolerates\n", gear,
               (unsigned long) ch224a_voltage_to_mv((ch224a_voltage_t) gear));
        return 1;
    }

    if (err != ESP_OK)
    {
        printf("Failed to request voltage (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Requested %lu mV, the supply only switches when it offers that gear\n",
           (unsigned long) ch224a_voltage_to_mv((ch224a_voltage_t) gear));
    return 0;
}

static void print_hall_reading(const drv5055_reading_t *reading)
{
    /* Field is carried in microtesla; print it as millitesla with three decimals */
    int32_t magnitude = reading->field_ut < 0 ? -reading->field_ut : reading->field_ut;

    printf("%s%ld.%03ld mT (%d mV)%s%s\n", reading->field_ut < 0 ? "-" : "", (long) (magnitude / 1000),
           (long) (magnitude % 1000), reading->millivolts, reading->magnet_present ? ", magnet" : "",
           reading->saturated ? ", SATURATED" : "");
}

static int cmd_hall(int argc, char **argv)
{
    drv5055_reading_t reading;

    esp_err_t err = drv5055_read(s_hall_handle, &reading);
    if (err != ESP_OK)
    {
        printf("Failed to read the Hall sensor (%s)\n", esp_err_to_name(err));
        return 1;
    }

    print_hall_reading(&reading);

    if (reading.saturated)
    {
        printf("Output is outside the linear window, the magnet is too strong or too close\n");
    }

    return 0;
}

static int cmd_hall_zero(int argc, char **argv)
{
    int zero_mv;

    esp_err_t err = drv5055_calibrate_zero(s_hall_handle);
    if (err != ESP_OK)
    {
        printf("Failed to calibrate (%s)\n", esp_err_to_name(err));
        return 1;
    }

    drv5055_get_zero_millivolts(s_hall_handle, &zero_mv);
    printf("Zero-field reference set to %d mV\n", zero_mv);
    return 0;
}

static int cmd_hall_watch(int argc, char **argv)
{
    drv5055_reading_t reading;
    int32_t           min_ut      = INT32_MAX;
    int32_t           max_ut      = INT32_MIN;
    bool              was_present = false;
    bool              first       = true;
    TickType_t        last_report = 0;

    int nerrors = arg_parse(argc, argv, (void **) &s_hall_watch_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_hall_watch_args.end, argv[0]);
        return 1;
    }

    int seconds = s_hall_watch_args.seconds->ival[0];
    if (seconds < HALL_WATCH_MIN_SECONDS || seconds > HALL_WATCH_MAX_SECONDS)
    {
        printf("Duration must be between %d and %d seconds\n", HALL_WATCH_MIN_SECONDS, HALL_WATCH_MAX_SECONDS);
        return 1;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t end   = start + pdMS_TO_TICKS(seconds * 1000);

    while (xTaskGetTickCount() < end)
    {
        esp_err_t err = drv5055_read(s_hall_handle, &reading);
        if (err != ESP_OK)
        {
            printf("Failed to read the Hall sensor (%s)\n", esp_err_to_name(err));
            return 1;
        }

        if (reading.field_ut < min_ut)
        {
            min_ut = reading.field_ut;
        }

        if (reading.field_ut > max_ut)
        {
            max_ut = reading.field_ut;
        }

        TickType_t now = xTaskGetTickCount();

        /* Report on every detection edge, and otherwise at a rate that stays readable */
        if (first || reading.magnet_present != was_present ||
            (now - last_report) >= pdMS_TO_TICKS(HALL_WATCH_REPORT_MS))
        {
            print_hall_reading(&reading);
            last_report = now;
            first       = false;
        }

        was_present = reading.magnet_present;

        vTaskDelay(pdMS_TO_TICKS(HALL_WATCH_POLL_MS));
    }

    printf("Range over %d s: %ld uT to %ld uT\n", seconds, (long) min_ut, (long) max_ut);
    return 0;
}

static int cmd_hall_threshold(int argc, char **argv)
{
    int32_t threshold_ut;
    int32_t hysteresis_ut;

    int nerrors = arg_parse(argc, argv, (void **) &s_hall_threshold_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_hall_threshold_args.end, argv[0]);
        return 1;
    }

    if (s_hall_threshold_args.threshold->count > 0)
    {
        threshold_ut = s_hall_threshold_args.threshold->ival[0];

        if (s_hall_threshold_args.hysteresis->count > 0)
        {
            hysteresis_ut = s_hall_threshold_args.hysteresis->ival[0];
        }
        else
        {
            drv5055_get_threshold(s_hall_handle, NULL, &hysteresis_ut);
        }

        esp_err_t err = drv5055_set_threshold(s_hall_handle, threshold_ut, hysteresis_ut);
        if (err != ESP_OK)
        {
            printf("Threshold must be positive and larger than the hysteresis\n");
            return 1;
        }
    }

    drv5055_get_threshold(s_hall_handle, &threshold_ut, &hysteresis_ut);
    printf("Threshold %ld uT, hysteresis %ld uT\n", (long) threshold_ut, (long) hysteresis_ut);
    return 0;
}

static void print_supply_reading(const vsense_reading_t *reading)
{
    printf("%lu.%03lu V (%d mV at the tap)%s%s\n", (unsigned long) (reading->millivolts / 1000),
           (unsigned long) (reading->millivolts % 1000), reading->pin_millivolts,
           reading->in_range ? "" : ", OUT OF RANGE", reading->clipped ? ", CLIPPED" : "");
}

static int cmd_supply(int argc, char **argv)
{
    vsense_reading_t reading;
    uint32_t         max_millivolts;

    esp_err_t err = vsense_read(s_supply_handle, &reading);
    if (err != ESP_OK)
    {
        printf("Failed to read the rail voltage (%s)\n", esp_err_to_name(err));
        return 1;
    }

    print_supply_reading(&reading);

    if (reading.clipped)
    {
        vsense_get_max_millivolts(s_supply_handle, &max_millivolts);
        printf("The ADC is saturated, the rail is above %lu mV\n", (unsigned long) max_millivolts);
    }

    return 0;
}

static int cmd_supply_calibrate(int argc, char **argv)
{
    uint32_t scale_ppm;

    int nerrors = arg_parse(argc, argv, (void **) &s_supply_calibrate_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_supply_calibrate_args.end, argv[0]);
        return 1;
    }

    int millivolts = s_supply_calibrate_args.millivolts->ival[0];
    if (millivolts <= 0)
    {
        printf("Measured voltage must be positive\n");
        return 1;
    }

    esp_err_t err = vsense_calibrate(s_supply_handle, (uint32_t) millivolts);
    if (err != ESP_OK)
    {
        printf("Failed to calibrate (%s), see the log for the reason\n", esp_err_to_name(err));
        return 1;
    }

    vsense_get_scale_ppm(s_supply_handle, &scale_ppm);
    printf("Gain correction now %lu ppm\n", (unsigned long) scale_ppm);
    return 0;
}

static int cmd_supply_range(int argc, char **argv)
{
    uint32_t undervoltage_mv;
    uint32_t overvoltage_mv;
    uint32_t hysteresis_mv;

    int nerrors = arg_parse(argc, argv, (void **) &s_supply_range_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_supply_range_args.end, argv[0]);
        return 1;
    }

    if (s_supply_range_args.undervoltage->count > 0)
    {
        if (s_supply_range_args.overvoltage->count == 0)
        {
            printf("Both limits are needed, pass 0 to disable one of them\n");
            return 1;
        }

        if (s_supply_range_args.undervoltage->ival[0] < 0 || s_supply_range_args.overvoltage->ival[0] < 0 ||
            (s_supply_range_args.hysteresis->count > 0 && s_supply_range_args.hysteresis->ival[0] < 0))
        {
            printf("Limits and hysteresis cannot be negative\n");
            return 1;
        }

        undervoltage_mv = (uint32_t) s_supply_range_args.undervoltage->ival[0];
        overvoltage_mv  = (uint32_t) s_supply_range_args.overvoltage->ival[0];

        if (s_supply_range_args.hysteresis->count > 0)
        {
            hysteresis_mv = (uint32_t) s_supply_range_args.hysteresis->ival[0];
        }
        else
        {
            vsense_get_range(s_supply_handle, NULL, NULL, &hysteresis_mv);
        }

        esp_err_t err = vsense_set_range(s_supply_handle, undervoltage_mv, overvoltage_mv, hysteresis_mv);
        if (err != ESP_OK)
        {
            printf("The window must be wider than the hysteresis\n");
            return 1;
        }
    }

    vsense_get_range(s_supply_handle, &undervoltage_mv, &overvoltage_mv, &hysteresis_mv);
    printf("Undervoltage %lu mV, overvoltage %lu mV, hysteresis %lu mV\n", (unsigned long) undervoltage_mv,
           (unsigned long) overvoltage_mv, (unsigned long) hysteresis_mv);
    return 0;
}

static void print_move_result(const dispenser_result_t *result)
{
    if (result->slot == DISPENSER_SLOT_NONE)
    {
        printf("Parked on an unmapped magnet");
    }
    else
    {
        printf("Parked on slot %d%s", result->slot, result->slot == DISPENSER_SLOT_HOME ? " (home)" : "");
    }

    printf(", peak ");
    print_field_mt(result->peak_ut);
    printf(", resting ");
    print_field_mt(result->field_ut);
    printf(", after %lu ms", (unsigned long) result->elapsed_ms);

    if (result->magnets_passed > 0)
    {
        printf(", %lu magnet%s passed", (unsigned long) result->magnets_passed, result->magnets_passed == 1 ? "" : "s");
    }

    printf("\n");
}

static void print_field_mt(int32_t field_ut)
{
    int32_t magnitude = field_ut < 0 ? -field_ut : field_ut;

    printf("%s%ld.%03ld mT", field_ut < 0 ? "-" : "", (long) (magnitude / 1000), (long) (magnitude % 1000));
}

static int report_move_error(const char *what, esp_err_t err)
{
    if (err == ESP_ERR_TIMEOUT)
    {
        printf("Timed out looking for a magnet, the drum may be jammed or the motor may be too slow\n");
    }
    else if (err == ESP_ERR_NOT_FOUND)
    {
        printf("Went past a full drum of magnets without finding a negative one, check the home magnet polarity\n");
    }
    else
    {
        printf("Failed to %s (%s)\n", what, esp_err_to_name(err));
    }

    return 1;
}

static int cmd_home(int argc, char **argv)
{
    dispenser_result_t result;

    esp_err_t err = dispenser_home(s_drum_handle, &result);
    if (err != ESP_OK)
    {
        return report_move_error("home the drum", err);
    }

    if (result.already_there)
    {
        printf("Already at home\n");
        return 0;
    }

    print_move_result(&result);
    return 0;
}

static int cmd_next(int argc, char **argv)
{
    dispenser_result_t result;

    esp_err_t err = dispenser_move(s_drum_handle, DISPENSER_MOVE_ADVANCE, &s_manual_chime, &result);
    if (err != ESP_OK)
    {
        return report_move_error("advance the drum", err);
    }

    print_move_result(&result);
    return 0;
}

static int cmd_prev(int argc, char **argv)
{
    dispenser_result_t result;

    esp_err_t err = dispenser_move(s_drum_handle, DISPENSER_MOVE_RETREAT, &s_manual_chime, &result);
    if (err != ESP_OK)
    {
        return report_move_error("turn the drum back", err);
    }

    print_move_result(&result);
    return 0;
}

static int cmd_dispense_speed(int argc, char **argv)
{
    uint32_t speed_pct;

    int nerrors = arg_parse(argc, argv, (void **) &s_dispense_speed_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_dispense_speed_args.end, argv[0]);
        return 1;
    }

    if (s_dispense_speed_args.value->count > 0)
    {
        int speed = s_dispense_speed_args.value->ival[0];
        if (speed < 1 || speed > 100)
        {
            printf("Speed must be between 1 and 100\n");
            return 1;
        }

        esp_err_t err = dispenser_set_travel_speed(s_drum_handle, (uint32_t) speed);
        if (err != ESP_OK)
        {
            printf("Failed to set the travel speed (%s)\n", esp_err_to_name(err));
            return 1;
        }
    }

    dispenser_get_travel_speed(s_drum_handle, &speed_pct);
    printf("Travel speed: %lu%%\n", (unsigned long) speed_pct);
    return 0;
}

static int cmd_schedule(int argc, char **argv)
{
    const scheduler_slot_t *slots      = NULL;
    size_t                  slot_count = 0;
    scheduler_slot_t        next;

    if (s_schedule_handle == NULL)
    {
        printf("No dispensing schedule, the RTC is unavailable\n");
        return 1;
    }

    scheduler_get_slots(s_schedule_handle, &slots, &slot_count);

    printf("Dispensing at");
    for (size_t i = 0; i < slot_count; i++)
    {
        printf("%s %02d:%02d", i == 0 ? "" : ",", slots[i].hour, slots[i].minute);
    }
    printf("\n");

    esp_err_t err = scheduler_get_next_slot(s_schedule_handle, &next);
    if (err != ESP_OK)
    {
        printf("Failed to read the armed alarm (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Alarm armed for %02d:%02d\n", next.hour, next.minute);
    return 0;
}

static int cmd_ble(int argc, char **argv)
{
    ble_remote_status_t status;

    if (s_ble_handle == NULL)
    {
        printf("No phone link, the BLE peripheral failed to start\n");
        return 1;
    }

    esp_err_t err = ble_remote_get_status(s_ble_handle, &status);
    if (err != ESP_OK)
    {
        printf("Failed to read the link state (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Link: %s", status.connected ? "connected" : "advertising");
    if (status.connected)
    {
        printf(", %s", status.encrypted ? "encrypted" : "not encrypted yet");
    }
    printf("\n");

    if (status.pairing_open)
    {
        printf("Pairing: open, %lu s left\n", (unsigned long) (status.pairing_left_ms / 1000));
    }
    else
    {
        printf("Pairing: shut, run ble_pair to re-open it\n");
    }

    printf("Bonds: %lu\n", (unsigned long) status.bond_count);
    printf("Dispenser: %s\n", status.busy ? "busy" : "idle");
    return 0;
}

static int cmd_ble_pair(int argc, char **argv)
{
    if (s_ble_handle == NULL)
    {
        printf("No phone link, the BLE peripheral failed to start\n");
        return 1;
    }

    esp_err_t err = ble_remote_open_pairing(s_ble_handle);
    if (err != ESP_OK)
    {
        printf("Failed to open the pairing window (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Pairing window open\n");
    return 0;
}

static int cmd_ble_forget(int argc, char **argv)
{
    if (s_ble_handle == NULL)
    {
        printf("No phone link, the BLE peripheral failed to start\n");
        return 1;
    }

    esp_err_t err = ble_remote_forget_bonds(s_ble_handle);
    if (err != ESP_OK)
    {
        printf("Failed to clear the bonds (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("All bonds deleted, forget the dispenser on the phone as well\n");
    return 0;
}

static esp_err_t register_drum_calibrate_command(void)
{
    s_drum_calibrate_args.revolutions = arg_int0("r", "revolutions", "<n>", "Full turns to average over (default 3)");
    s_drum_calibrate_args.speed       = arg_int0("s", "speed", "<pct>", "Motor speed while turning");
    s_drum_calibrate_args.gate        = arg_int0("g", "gate", "<uT>", "Field that opens a magnet pass (default 3000)");
    s_drum_calibrate_args.forward     = arg_lit0("f", "forward", "Turn the dispensing way, which empties the drum");
    s_drum_calibrate_args.home_weak =
        arg_lit0(NULL, "home-weak", "Slots 0 and 1 read weaker (default: they read stronger)");
    s_drum_calibrate_args.zero    = arg_lit0("z", "zero", "Recapture the zero-field reference between magnets first");
    s_drum_calibrate_args.dry_run = arg_lit0("n", "dry-run", "Measure and report without storing the map");
    s_drum_calibrate_args.end     = arg_end(7);

    const esp_console_cmd_t cmd = {
        .command  = "drum_calibrate",
        .help     = "Turn the drum a few times, learn the field level of each slot and store the map",
        .hint     = NULL,
        .func     = &cmd_drum_calibrate,
        .argtable = &s_drum_calibrate_args,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_drum_map_command(void)
{
    const esp_console_cmd_t cmd = {
        .command  = "drum_map",
        .help     = "Show the active slot map",
        .hint     = NULL,
        .func     = &cmd_drum_map,
        .argtable = NULL,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_drum_slot_command(void)
{
    const esp_console_cmd_t cmd = {
        .command  = "drum_slot",
        .help     = "Show which slot is at the opening right now",
        .hint     = NULL,
        .func     = &cmd_drum_slot,
        .argtable = NULL,
    };

    return esp_console_cmd_register(&cmd);
}

static esp_err_t register_drum_forget_command(void)
{
    const esp_console_cmd_t cmd = {
        .command  = "drum_forget",
        .help     = "Erase the stored slot map",
        .hint     = NULL,
        .func     = &cmd_drum_forget,
        .argtable = NULL,
    };

    return esp_console_cmd_register(&cmd);
}

static int cmd_drum_calibrate(int argc, char **argv)
{
    dispenser_cal_config_t config = {0};
    dispenser_cal_result_t result;
    esp_err_t              err;

    int errors = arg_parse(argc, argv, (void **) &s_drum_calibrate_args);
    if (errors != 0)
    {
        arg_print_errors(stderr, s_drum_calibrate_args.end, argv[0]);
        return 1;
    }

    if (s_drum_calibrate_args.revolutions->count > 0)
    {
        config.revolutions = (uint32_t) s_drum_calibrate_args.revolutions->ival[0];
    }

    if (s_drum_calibrate_args.speed->count > 0)
    {
        config.speed_pct = (uint32_t) s_drum_calibrate_args.speed->ival[0];
    }

    if (s_drum_calibrate_args.gate->count > 0)
    {
        config.gate_ut = s_drum_calibrate_args.gate->ival[0];
    }

    config.dispensing_direction = s_drum_calibrate_args.forward->count > 0;
    config.home_pair_is_weaker  = s_drum_calibrate_args.home_weak->count > 0;
    config.recalibrate_zero     = s_drum_calibrate_args.zero->count > 0;
    config.skip_save            = s_drum_calibrate_args.dry_run->count > 0;

    printf("Turning the drum, this takes a moment...\n");

    err = dispenser_calibrate(s_drum_handle, &config, &result);
    if (err != ESP_OK)
    {
        printf("Calibration failed (%s), see the log above for what went wrong\n", esp_err_to_name(err));
        return 1;
    }

    print_calibration(&result);
    printf("Baseline between magnets %ld uT, %lu passes captured, map %s\n", (long) result.baseline_ut,
           (unsigned long) result.peaks_seen, result.saved ? "stored" : "NOT stored (dry run)");
    return 0;
}

static int cmd_drum_map(int argc, char **argv)
{
    dispenser_cal_result_t result;

    esp_err_t err = dispenser_calibration_get(s_drum_handle, &result);
    if (err == ESP_ERR_INVALID_STATE)
    {
        printf("No slot map, run drum_calibrate\n");
        return 1;
    }

    if (err != ESP_OK)
    {
        printf("Failed to read the map (%s)\n", esp_err_to_name(err));
        return 1;
    }

    print_calibration(&result);
    return 0;
}

/*
 * Both answers, because they disagree by design: the drum parks past the peak,
 * so classifying the field as it stands can name a weaker slot than the magnet
 * the drum is on. The remembered one is the answer to trust; the live one is
 * here to show how far past the peak the drum came to rest.
 */
static int cmd_drum_slot(int argc, char **argv)
{
    int     slot;
    int     live = DISPENSER_SLOT_NONE;
    int32_t field_ut;

    esp_err_t err = dispenser_get_slot(s_drum_handle, &slot);
    if (err == ESP_ERR_INVALID_STATE)
    {
        printf("No slot map, run drum_calibrate\n");
        return 1;
    }

    if (err != ESP_OK)
    {
        printf("Failed to read the slot (%s)\n", esp_err_to_name(err));
        return 1;
    }

    if (slot == DISPENSER_SLOT_NONE)
    {
        printf("Position unknown: move the drum once to establish it\n");
    }
    else
    {
        printf("Slot %d%s\n", slot, slot == DISPENSER_SLOT_HOME ? " (home)" : "");
    }

    if (drv5055_read_field_ut(s_hall_handle, &field_ut) == ESP_OK)
    {
        dispenser_classify_field(s_drum_handle, field_ut, &live);

        printf("  field here ");
        print_field_mt(field_ut);
        if (live == DISPENSER_SLOT_NONE)
        {
            printf(", under the detection gate\n");
        }
        else
        {
            printf(", which on its own would read as slot %d\n", live);
        }
    }

    return 0;
}

static esp_err_t register_locate_command(void)
{
    const esp_console_cmd_t cmd = {
        .command  = "locate",
        .help     = "Back the drum off its magnet and turn forward over it again to find which slot it is on",
        .hint     = NULL,
        .func     = &cmd_locate,
        .argtable = NULL,
    };

    return esp_console_cmd_register(&cmd);
}

static int cmd_locate(int argc, char **argv)
{
    dispenser_result_t result;

    esp_err_t err = dispenser_find_position(s_drum_handle, &result);
    if (err == ESP_ERR_INVALID_STATE)
    {
        printf("No slot map, run drum_calibrate first\n");
        return 1;
    }

    if (err != ESP_OK)
    {
        return report_move_error("work out which slot the drum is on", err);
    }

    if (result.already_there)
    {
        printf("Position already known: slot %d\n", result.slot);
        return 0;
    }

    print_move_result(&result);
    return 0;
}

static esp_err_t register_slot_command(void)
{
    s_slot_args.number = arg_int1(NULL, NULL, "<n>", "Slot to park at the opening (0 is home)");
    s_slot_args.end    = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command  = "slot",
        .help     = "Turn the drum on until the given slot is at the opening, dispensing what it crosses",
        .hint     = NULL,
        .func     = &cmd_slot,
        .argtable = &s_slot_args,
    };

    return esp_console_cmd_register(&cmd);
}

static int cmd_slot(int argc, char **argv)
{
    dispenser_result_t result;
    esp_err_t          err;
    int                slot;

    int errors = arg_parse(argc, argv, (void **) &s_slot_args);
    if (errors != 0)
    {
        arg_print_errors(stderr, s_slot_args.end, argv[0]);
        return 1;
    }

    slot = s_slot_args.number->ival[0];

    err = dispenser_go_to_slot(s_drum_handle, slot, &s_manual_chime, &result);
    if (err == ESP_ERR_INVALID_ARG)
    {
        printf("Slot must be 0 to %d\n", DISPENSER_SLOT_COUNT - 1);
        return 1;
    }

    if (err == ESP_ERR_INVALID_STATE)
    {
        printf("No slot map, run drum_calibrate before asking for a slot by number\n");
        return 1;
    }

    if (err != ESP_OK)
    {
        return report_move_error("reach that slot", err);
    }

    if (result.already_there)
    {
        printf("Already on slot %d\n", slot);
        return 0;
    }

    print_move_result(&result);
    return 0;
}

static int cmd_drum_forget(int argc, char **argv)
{
    esp_err_t err = dispenser_calibration_erase(s_drum_handle);
    if (err != ESP_OK)
    {
        printf("Failed to erase the map (%s)\n", esp_err_to_name(err));
        return 1;
    }

    printf("Slot map erased\n");
    return 0;
}

/*
 * The levels are printed in slot order with their scatter, because a slot whose
 * scatter approaches the margin is the one that will misread first.
 */
static void print_calibration(const dispenser_cal_result_t *result)
{
    int i;

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        printf("  slot %d%-7s %7ld uT  +/-%ld uT\n", i, i == DISPENSER_SLOT_HOME ? " (home)" : "",
               (long) result->level_ut[i], (long) (result->spread_ut[i] / 2));
    }

    printf("  boundaries  %ld / %ld / %ld uT\n", (long) result->boundary_ut[0], (long) result->boundary_ut[1],
           (long) result->boundary_ut[2]);
    printf("  margin %ld uT, gate %ld uT, zero reference %ld mV\n", (long) result->margin_ut, (long) result->gate_ut,
           (long) result->zero_mv);
}
