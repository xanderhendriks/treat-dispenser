#include "dispenser.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define DISPENSER_DEFAULT_SPEED_PCT  60
#define DISPENSER_DEFAULT_POLL_MS    10
#define DISPENSER_DEFAULT_TIMEOUT_MS 5000
#define DISPENSER_DEFAULT_BRAKE_MS   150

#define DISPENSER_MAX_SPEED_PCT 100

/*
 * Bound on how far dispenser_home() keeps hunting. A drum holds far fewer
 * magnets than this, so hitting the bound means the home magnet is missing or
 * mounted the wrong way round rather than that the search needs more time.
 */
#define DISPENSER_MAX_MAGNETS_PASSED 32

/*
 * How far the field has to fall back from its highest reading before a magnet
 * counts as crossed. The detection threshold trips low on a magnet's flank, far
 * below its peak, so a move that stopped there would both leave the drum short
 * of the slot centre and read a field too weak to tell a near magnet from a far
 * one. Waiting for the peak fixes the identification outright and stops the drum
 * a fraction past centre instead of well before it.
 *
 * The margin is a fraction of the peak so that it scales with the near and far
 * magnets alike, with a floor to keep ADC noise from ending a pass early.
 */
#define DISPENSER_PEAK_DROP_PERCENT 5
#define DISPENSER_PEAK_DROP_MIN_UT  500

/*
 * Calibration of the slot map. Three revolutions is enough to average out the
 * scatter of a hand-mounted magnet without making the run tedious, and the gate
 * sits far below the weakest magnet so that even the far pair opens a pass.
 */
#define DISPENSER_CAL_DEFAULT_REVOLUTIONS 3
#define DISPENSER_CAL_MAX_REVOLUTIONS     8
#define DISPENSER_CAL_MAX_PEAKS           (DISPENSER_SLOT_COUNT * DISPENSER_CAL_MAX_REVOLUTIONS)
#define DISPENSER_CAL_DEFAULT_GATE_UT     3000

/*
 * How far apart the levels have to come out for the map to be trustworthy. The
 * separation is what any two neighbouring levels need; the tier gap is what the
 * two mounting heights need, and is deliberately larger because telling the
 * heights apart is what anchors the map to the drum.
 */
#define DISPENSER_CAL_MIN_SEPARATION_UT 3000
#define DISPENSER_CAL_MIN_TIER_GAP_UT   4000

/*
 * Extra run after the field has fallen away while backing off to find the
 * position, so that the sweep back over the magnet starts from clear air and
 * measures the whole rise rather than joining it part way up.
 */
#define DISPENSER_FIND_BACKOFF_MS 150

/*
 * How much field has to still be showing for a remembered position to be
 * believed. A move parks the drum past the peak so the field has already fallen
 * well off it, which is why this is far below the detection gate rather than a
 * fraction of a magnet; what it has to rule out is a drum turned away from its
 * magnet by hand, which reads next to nothing.
 */
#define DISPENSER_PARKED_MIN_UT 2500

/* A between-magnet field bigger than this means the zero-field reference drifted */
#define DISPENSER_CAL_BASELINE_WARN_UT 1500

#define DISPENSER_NVS_NAMESPACE "dispenser"
#define DISPENSER_NVS_KEY_MAP   "drum_map"

/* Bump the version whenever the layout of dispenser_stored_map_t changes */
#define DISPENSER_MAP_MAGIC   0x4452554Du
#define DISPENSER_MAP_VERSION 1u

/*
 * What actually goes into non-volatile storage. Every field is a fixed-width
 * 32-bit type so the blob has no padding and means the same thing to any build.
 * The boundaries are left out because they follow from the levels.
 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t slot_count;
    int32_t  zero_mv;
    int32_t  gate_ut;
    int32_t  level_ut[DISPENSER_SLOT_COUNT];
} dispenser_stored_map_t;

typedef struct dispenser_t
{
    drv8871_handle_t   motor;
    drv5055_handle_t   hall;
    max98357a_handle_t audio;

    dispenser_chime_t home_chime;
    dispenser_chime_t advance_chime;
    dispenser_chime_t retreat_chime;

    drv8871_direction_t direction;
    uint32_t            speed_pct;
    uint32_t            poll_interval_ms;
    uint32_t            timeout_ms;
    uint32_t            brake_ms;

    /* Slot map, learned by dispenser_calibrate() and reloaded from NVS at init */
    bool     cal_valid;
    int32_t  cal_level_ut[DISPENSER_SLOT_COUNT];     /* signed level, indexed by slot */
    int32_t  cal_spread_ut[DISPENSER_SLOT_COUNT];    /* scatter across revolutions, indexed by slot */
    int32_t  cal_ordered_ut[DISPENSER_SLOT_COUNT];   /* the same levels, ascending */
    int      cal_ordered_slot[DISPENSER_SLOT_COUNT]; /* which slot each ascending level belongs to */
    int32_t  cal_boundary_ut[DISPENSER_SLOT_COUNT - 1];
    int32_t  cal_margin_ut;
    int32_t  cal_gate_ut;
    int      cal_zero_mv;
    uint32_t cal_peaks_seen;

    /*
     * Slot the drum was left on by the last move, judged from that magnet's peak
     * rather than from where the drum came to rest. DISPENSER_SLOT_NONE until a
     * move establishes it.
     */
    int cal_parked_slot;
} dispenser_ctx_t;

static const char *TAG = "dispenser";

static esp_err_t dispenser_seek(dispenser_ctx_t *ctx, dispenser_move_t move, int target_slot,
                                dispenser_result_t *out_result);
static void      dispenser_play_chime(dispenser_ctx_t *ctx, const dispenser_chime_t *chime);

static const dispenser_chime_t *dispenser_move_chime(const dispenser_ctx_t *ctx, dispenser_move_t move);
static drv8871_direction_t      dispenser_move_direction(const dispenser_ctx_t *ctx, dispenser_move_t move);

static esp_err_t dispenser_start_motor(dispenser_ctx_t *ctx, dispenser_move_t move);
static esp_err_t dispenser_stop_motor(dispenser_ctx_t *ctx, bool brake);
static bool      dispenser_magnet_matches(const dispenser_ctx_t *ctx, int target_slot, int32_t field_ut);
static bool      dispenser_within_magnet(dispenser_ctx_t *ctx, int32_t field_ut);
static int32_t   dispenser_magnitude(int32_t field_ut);
static void      dispenser_sort_pairs(int32_t *keys, int32_t *tags, size_t count);
static int       dispenser_slot_for_field(const dispenser_ctx_t *ctx, int32_t field_ut);
static int32_t   dispenser_peak_drop(int32_t peak_ut);
static int       dispenser_parked_slot(const dispenser_ctx_t *ctx, int32_t field_ut);
static bool      dispenser_already_at(const dispenser_ctx_t *ctx, int target_slot, const drv5055_reading_t *reading);
static esp_err_t dispenser_drive_clear(dispenser_ctx_t *ctx, dispenser_move_t move, uint32_t extra_ms);

static esp_err_t dispenser_zero_between_magnets(dispenser_ctx_t *ctx, const dispenser_cal_config_t *cfg);
static esp_err_t dispenser_capture_peaks(dispenser_ctx_t *ctx, const dispenser_cal_config_t *cfg, int32_t *peaks,
                                         uint32_t want, dispenser_cal_result_t *result);
static esp_err_t dispenser_derive_map(dispenser_ctx_t *ctx, const dispenser_cal_config_t *cfg, const int32_t *peaks,
                                      uint32_t count, dispenser_cal_result_t *result);
static esp_err_t dispenser_activate_map(dispenser_ctx_t *ctx, const int32_t *level_ut, int32_t gate_ut, int32_t zero_mv,
                                        dispenser_cal_result_t *result);
static esp_err_t dispenser_save_map(dispenser_ctx_t *ctx);
static esp_err_t dispenser_load_map(dispenser_ctx_t *ctx);

esp_err_t dispenser_init(const dispenser_config_t *config, dispenser_handle_t *out_handle)
{
    dispenser_ctx_t *ctx;
    esp_err_t        err;

    if (!config || !config->motor_handle || !config->hall_handle || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->travel_speed_pct > DISPENSER_MAX_SPEED_PCT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->direction != DRV8871_DIRECTION_FORWARD && config->direction != DRV8871_DIRECTION_REVERSE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(dispenser_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->motor            = config->motor_handle;
    ctx->hall             = config->hall_handle;
    ctx->audio            = config->audio_handle;
    ctx->home_chime       = config->home_chime;
    ctx->advance_chime    = config->advance_chime;
    ctx->retreat_chime    = config->retreat_chime;
    ctx->direction        = config->direction;
    ctx->speed_pct        = config->travel_speed_pct ? config->travel_speed_pct : DISPENSER_DEFAULT_SPEED_PCT;
    ctx->poll_interval_ms = config->poll_interval_ms ? config->poll_interval_ms : DISPENSER_DEFAULT_POLL_MS;
    ctx->timeout_ms       = config->timeout_ms ? config->timeout_ms : DISPENSER_DEFAULT_TIMEOUT_MS;
    ctx->brake_ms         = config->brake_ms ? config->brake_ms : DISPENSER_DEFAULT_BRAKE_MS;

    /* calloc left this 0, which is the home slot rather than "nowhere known" */
    ctx->cal_parked_slot = DISPENSER_SLOT_NONE;

    err = dispenser_load_map(ctx);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded the slot map (home %ld, 1 %ld, 2 %ld, 3 %ld uT, gate %ld uT)",
                 (long) ctx->cal_level_ut[0], (long) ctx->cal_level_ut[1], (long) ctx->cal_level_ut[2],
                 (long) ctx->cal_level_ut[3], (long) ctx->cal_gate_ut);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "No slot map stored; run dispenser_calibrate() before trusting the position");
    }
    else
    {
        ESP_LOGW(TAG, "Could not load the slot map (%s)", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Initialized (%s at %lu%%, %lu ms per magnet)",
             ctx->direction == DRV8871_DIRECTION_REVERSE ? "reverse" : "forward", (unsigned long) ctx->speed_pct,
             (unsigned long) ctx->timeout_ms);

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t dispenser_home(dispenser_handle_t handle, dispenser_result_t *out_result)
{
    return dispenser_move(handle, DISPENSER_MOVE_HOME, NULL, out_result);
}

esp_err_t dispenser_advance(dispenser_handle_t handle, dispenser_result_t *out_result)
{
    return dispenser_move(handle, DISPENSER_MOVE_ADVANCE, NULL, out_result);
}

esp_err_t dispenser_retreat(dispenser_handle_t handle, dispenser_result_t *out_result)
{
    return dispenser_move(handle, DISPENSER_MOVE_RETREAT, NULL, out_result);
}

esp_err_t dispenser_move(dispenser_handle_t handle, dispenser_move_t move, const dispenser_chime_t *chime,
                         dispenser_result_t *out_result)
{
    esp_err_t err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (move != DISPENSER_MOVE_HOME && move != DISPENSER_MOVE_ADVANCE && move != DISPENSER_MOVE_RETREAT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Homing hunts for slot 0; a plain advance or retreat takes whatever comes next */
    err = dispenser_seek(handle, move, move == DISPENSER_MOVE_HOME ? DISPENSER_SLOT_HOME : DISPENSER_SLOT_NONE,
                         out_result);
    if (err == ESP_OK)
    {
        dispenser_play_chime(handle, chime ? chime : dispenser_move_chime(handle, move));
    }

    return err;
}

esp_err_t dispenser_go_to_slot(dispenser_handle_t handle, int slot, const dispenser_chime_t *chime,
                               dispenser_result_t *out_result)
{
    esp_err_t err;

    if (!handle || slot < 0 || slot >= DISPENSER_SLOT_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Without a map every magnet looks alike, so a slot cannot be asked for by number */
    if (!handle->cal_valid)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Always the dispensing way, however far round that is. A move stops just
     * past the peak of the magnet it settles on, so approaching from one side
     * only keeps that overshoot and the gear backlash on the same side and a
     * slot lands in the same place every time. Picking the shorter way round
     * would leave a slot sitting a little differently depending on which side
     * the drum arrived from.
     */
    ESP_LOGI(TAG, "Going to slot %d, turning on", slot);

    err = dispenser_seek(handle, DISPENSER_MOVE_ADVANCE, slot, out_result);
    if (err == ESP_OK)
    {
        dispenser_play_chime(handle, chime ? chime : dispenser_move_chime(handle, DISPENSER_MOVE_ADVANCE));
    }

    return err;
}

esp_err_t dispenser_is_home(dispenser_handle_t handle, bool *out_home)
{
    esp_err_t         err;
    drv5055_reading_t reading;

    if (!handle || !out_home)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = drv5055_read(handle->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_home = dispenser_already_at(handle, DISPENSER_SLOT_HOME, &reading);
    return ESP_OK;
}

esp_err_t dispenser_set_travel_speed(dispenser_handle_t handle, uint32_t speed_pct)
{
    if (!handle || speed_pct == 0 || speed_pct > DISPENSER_MAX_SPEED_PCT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->speed_pct = speed_pct;
    return ESP_OK;
}

esp_err_t dispenser_get_travel_speed(dispenser_handle_t handle, uint32_t *out_speed_pct)
{
    if (!handle || !out_speed_pct)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_speed_pct = handle->speed_pct;
    return ESP_OK;
}

esp_err_t dispenser_calibrate(dispenser_handle_t handle, const dispenser_cal_config_t *config,
                              dispenser_cal_result_t *out_result)
{
    dispenser_cal_config_t cfg = {0};
    dispenser_cal_result_t result;
    int32_t                peaks[DISPENSER_CAL_MAX_PEAKS];
    uint32_t               revolutions;
    uint32_t               want;
    esp_err_t              err;
    size_t                 i;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config)
    {
        cfg = *config;
    }

    revolutions = cfg.revolutions ? cfg.revolutions : DISPENSER_CAL_DEFAULT_REVOLUTIONS;
    if (revolutions > DISPENSER_CAL_MAX_REVOLUTIONS || cfg.speed_pct > DISPENSER_MAX_SPEED_PCT || cfg.gate_ut < 0 ||
        cfg.release_ut < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg.gate_ut == 0)
    {
        cfg.gate_ut = DISPENSER_CAL_DEFAULT_GATE_UT;
    }

    if (cfg.release_ut == 0)
    {
        cfg.release_ut = cfg.gate_ut / 2;
    }

    /* Without hysteresis a magnet sitting on the gate would be counted over and over */
    if (cfg.release_ut >= cfg.gate_ut)
    {
        return ESP_ERR_INVALID_ARG;
    }

    want = DISPENSER_SLOT_COUNT * revolutions;

    if (cfg.recalibrate_zero)
    {
        err = dispenser_zero_between_magnets(handle, &cfg);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    memset(&result, 0, sizeof(result));

    ESP_LOGI(TAG, "Calibrating over %lu revolutions (%lu magnet passes), turning the %s way",
             (unsigned long) revolutions, (unsigned long) want,
             cfg.dispensing_direction ? "dispensing" : "non-dispensing");

    err = dispenser_capture_peaks(handle, &cfg, peaks, want, &result);
    if (err != ESP_OK)
    {
        return err;
    }

    err = dispenser_derive_map(handle, &cfg, peaks, want, &result);
    if (err != ESP_OK)
    {
        return err;
    }

    if (!cfg.skip_save)
    {
        err = dispenser_save_map(handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Measured the slot map but could not store it (%s)", esp_err_to_name(err));
            return err;
        }

        result.saved = true;
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        handle->cal_spread_ut[i] = result.spread_ut[i];
    }

    handle->cal_peaks_seen = result.peaks_seen;

    ESP_LOGI(TAG, "Slot map: home %ld, 1 %ld, 2 %ld, 3 %ld uT (margin %ld uT, gate %ld uT)%s",
             (long) result.level_ut[0], (long) result.level_ut[1], (long) result.level_ut[2], (long) result.level_ut[3],
             (long) result.margin_ut, (long) result.gate_ut, result.saved ? ", stored" : ", not stored");

    if (out_result)
    {
        *out_result = result;
    }

    return ESP_OK;
}

esp_err_t dispenser_calibration_load(dispenser_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return dispenser_load_map(handle);
}

esp_err_t dispenser_calibration_get(dispenser_handle_t handle, dispenser_cal_result_t *out_result)
{
    size_t i;

    if (!handle || !out_result)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->cal_valid)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(out_result, 0, sizeof(*out_result));

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        out_result->level_ut[i]  = handle->cal_level_ut[i];
        out_result->spread_ut[i] = handle->cal_spread_ut[i];
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT - 1; i++)
    {
        out_result->boundary_ut[i] = handle->cal_boundary_ut[i];
    }

    out_result->margin_ut  = handle->cal_margin_ut;
    out_result->gate_ut    = handle->cal_gate_ut;
    out_result->zero_mv    = handle->cal_zero_mv;
    out_result->peaks_seen = handle->cal_peaks_seen;
    out_result->saved      = true;

    return ESP_OK;
}

esp_err_t dispenser_calibration_erase(dispenser_handle_t handle)
{
    nvs_handle_t nvs;
    esp_err_t    err;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    handle->cal_valid       = false;
    handle->cal_parked_slot = DISPENSER_SLOT_NONE;

    err = nvs_open(DISPENSER_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        return ESP_OK;
    }

    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_erase_key(nvs, DISPENSER_NVS_KEY_MAP);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        err = ESP_OK;
    }

    if (err == ESP_OK)
    {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);

    ESP_LOGI(TAG, "Slot map erased, homing is back to hunting the one negative magnet");
    return err;
}

bool dispenser_is_calibrated(dispenser_handle_t handle)
{
    return handle && handle->cal_valid;
}

esp_err_t dispenser_find_position(dispenser_handle_t handle, dispenser_result_t *out_result)
{
    esp_err_t         err;
    drv5055_reading_t reading;

    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->cal_valid)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* Already know where it is, so there is nothing to find out and no reason to move */
    err = drv5055_read(handle->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    if (dispenser_parked_slot(handle, reading.field_ut) != DISPENSER_SLOT_NONE)
    {
        if (out_result)
        {
            dispenser_result_t result = {0};

            result.field_ut      = reading.field_ut;
            result.peak_ut       = reading.field_ut;
            result.slot          = handle->cal_parked_slot;
            result.already_there = true;

            *out_result = result;
        }

        return ESP_OK;
    }

    ESP_LOGI(TAG, "Finding the position: backing off, then turning forward over the magnet");

    /*
     * Back off first. Going forward from where the drum already sits would join
     * the magnet's field on its way down and never see a peak, so the only way
     * to measure one is to get clear and come at it again.
     */
    err = dispenser_drive_clear(handle, DISPENSER_MOVE_RETREAT, DISPENSER_FIND_BACKOFF_MS);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Forward over the same magnet, which stops on its peak and names the slot */
    return dispenser_seek(handle, DISPENSER_MOVE_ADVANCE, DISPENSER_SLOT_NONE, out_result);
}

esp_err_t dispenser_get_slot(dispenser_handle_t handle, int *out_slot)
{
    esp_err_t err;
    int32_t   field_ut;

    if (!handle || !out_slot)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->cal_valid)
    {
        return ESP_ERR_INVALID_STATE;
    }

    err = drv5055_read_field_ut(handle->hall, &field_ut);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_slot = dispenser_parked_slot(handle, field_ut);
    return ESP_OK;
}

esp_err_t dispenser_classify_field(dispenser_handle_t handle, int32_t field_ut, int *out_slot)
{
    if (!handle || !out_slot)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->cal_valid)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *out_slot = dispenser_slot_for_field(handle, field_ut);
    return ESP_OK;
}

/*
 * The chime is an acknowledgement, not part of the move: playback runs in its
 * own task and a failure to start it is logged rather than reported, so a mute
 * amplifier never turns a successful move into an error.
 */
static void dispenser_play_chime(dispenser_ctx_t *ctx, const dispenser_chime_t *chime)
{
    esp_err_t err;

    if (!ctx->audio || chime->repeat_count == 0)
    {
        return;
    }

    err = max98357a_play(ctx->audio, chime->melody, chime->repeat_count, chime->volume_pct);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to play the chime (%s)", esp_err_to_name(err));
    }
}

static const dispenser_chime_t *dispenser_move_chime(const dispenser_ctx_t *ctx, dispenser_move_t move)
{
    switch (move)
    {
        case DISPENSER_MOVE_HOME:
            return &ctx->home_chime;

        case DISPENSER_MOVE_RETREAT:
            return &ctx->retreat_chime;

        default:
            return &ctx->advance_chime;
    }
}

/*
 * Retreating is the configured dispensing direction run backwards; the drum has
 * no separate reverse speed or timeout because a magnet-to-magnet hop costs the
 * same either way.
 */
static drv8871_direction_t dispenser_move_direction(const dispenser_ctx_t *ctx, dispenser_move_t move)
{
    if (move != DISPENSER_MOVE_RETREAT)
    {
        return ctx->direction;
    }

    return ctx->direction == DRV8871_DIRECTION_FORWARD ? DRV8871_DIRECTION_REVERSE : DRV8871_DIRECTION_FORWARD;
}

/*
 * Whether the magnet in front of the sensor is the one a move is hunting for.
 *
 * A target of DISPENSER_SLOT_NONE takes the next magnet whatever it is, which is
 * what a plain advance or retreat wants. Otherwise, with a slot map in place it
 * is a lookup: the four magnets are mounted at two heights and two polarities,
 * so the field alone says which slot is at the opening.
 *
 * Without a map it falls back to the older arrangement, where home was the only
 * magnet turned round and so the only one reading negative. Nothing but home can
 * be named that way, so an uncalibrated drum can still home but cannot be sent
 * to a slot by number.
 */
static bool dispenser_magnet_matches(const dispenser_ctx_t *ctx, int target_slot, int32_t field_ut)
{
    if (target_slot == DISPENSER_SLOT_NONE)
    {
        return true;
    }

    if (ctx->cal_valid)
    {
        return dispenser_slot_for_field(ctx, field_ut) == target_slot;
    }

    return target_slot == DISPENSER_SLOT_HOME && field_ut < 0;
}

/*
 * Whether the drum is still close enough to a magnet for the next detection to
 * be that same magnet. The DRV5055 presence flag latches with hysteresis, so a
 * drum parked just under the trip point reads as no magnet even though it is
 * sitting on one; comparing against the lower hysteresis bound catches that and
 * keeps a move from stopping straight back where it started.
 */
static bool dispenser_within_magnet(dispenser_ctx_t *ctx, int32_t field_ut)
{
    int32_t threshold_ut  = 0;
    int32_t hysteresis_ut = 0;
    int32_t magnitude     = field_ut < 0 ? -field_ut : field_ut;

    if (drv5055_get_threshold(ctx->hall, &threshold_ut, &hysteresis_ut) != ESP_OK)
    {
        return false;
    }

    return magnitude > threshold_ut - hysteresis_ut;
}

/*
 * All three moves are the same hunt: drive off whatever magnet the drum is
 * parked on, then stop on the first magnet that satisfies the caller. Which way
 * the drum turns is the only difference between advancing and retreating, the
 * magnet the hunt settles on being whichever one comes past first. Detection
 * rides on the hysteresis the DRV5055 driver already applies, so a magnet counts
 * once on the way in and only counts again after the field has fallen away.
 *
 * The timeout is a budget per hop rather than for the whole move, so homing
 * past several magnets does not have to outrun a single deadline.
 */
static esp_err_t dispenser_seek(dispenser_ctx_t *ctx, dispenser_move_t move, int target_slot,
                                dispenser_result_t *out_result)
{
    esp_err_t          err;
    drv5055_reading_t  reading;
    dispenser_result_t result  = {0};
    int32_t            peak_ut = 0;
    int32_t            magnitude;
    bool               in_pass = false;
    bool               clear_first;
    TickType_t         start;
    TickType_t         deadline;

    result.slot = DISPENSER_SLOT_NONE;

    err = drv5055_read(ctx->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    if (dispenser_already_at(ctx, target_slot, &reading))
    {
        result.field_ut      = reading.field_ut;
        result.peak_ut       = reading.field_ut;
        result.slot          = ctx->cal_valid ? target_slot : DISPENSER_SLOT_NONE;
        result.already_there = true;

        ESP_LOGI(TAG, "Already parked on slot %d", target_slot);

        if (out_result)
        {
            *out_result = result;
        }

        return ESP_OK;
    }

    clear_first = reading.magnet_present || dispenser_within_magnet(ctx, reading.field_ut);

    err = dispenser_start_motor(ctx, move);
    if (err != ESP_OK)
    {
        return err;
    }

    start    = xTaskGetTickCount();
    deadline = start + pdMS_TO_TICKS(ctx->timeout_ms);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ctx->poll_interval_ms));

        err = drv5055_read(ctx->hall, &reading);
        if (err != ESP_OK)
        {
            dispenser_stop_motor(ctx, false);
            return err;
        }

        magnitude = dispenser_magnitude(reading.field_ut);

        if (clear_first)
        {
            /* Waiting for the field of the magnet the drum started on to fall away */
            if (!reading.magnet_present && !dispenser_within_magnet(ctx, reading.field_ut))
            {
                clear_first = false;
                deadline    = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);
            }
        }
        else if (!in_pass)
        {
            if (reading.magnet_present)
            {
                in_pass = true;
                peak_ut = reading.field_ut;
            }
        }
        else if (magnitude > dispenser_magnitude(peak_ut))
        {
            peak_ut = reading.field_ut;
        }
        else if (magnitude <= dispenser_magnitude(peak_ut) - dispenser_peak_drop(peak_ut))
        {
            /* Over the top of the magnet, so the peak now says which one it was */
            if (dispenser_magnet_matches(ctx, target_slot, peak_ut))
            {
                break;
            }

            /* Not the slot wanted: count it and carry on to the next one */
            result.magnets_passed++;
            if (result.magnets_passed >= DISPENSER_MAX_MAGNETS_PASSED)
            {
                dispenser_stop_motor(ctx, true);
                ctx->cal_parked_slot = DISPENSER_SLOT_NONE;
                ESP_LOGE(TAG, "No slot %d after %lu magnets", target_slot, (unsigned long) result.magnets_passed);
                return ESP_ERR_NOT_FOUND;
            }

            in_pass     = false;
            clear_first = true;
            deadline    = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);
        }

        if (xTaskGetTickCount() >= deadline)
        {
            dispenser_stop_motor(ctx, true);
            ctx->cal_parked_slot = DISPENSER_SLOT_NONE;
            ESP_LOGE(TAG, "No magnet within %lu ms, the drum may be jammed", (unsigned long) ctx->timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }

    result.elapsed_ms = (uint32_t) ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);

    err = dispenser_stop_motor(ctx, true);
    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * The slot comes from the peak, which was measured while the magnet was
     * centred on the sensor. The resting field is reported alongside it purely as
     * a diagnostic: braking carries the drum a little past centre, so it reads
     * lower than the peak and is the weaker of the two to judge a slot by.
     */
    result.peak_ut = peak_ut;
    result.slot    = ctx->cal_valid ? dispenser_slot_for_field(ctx, peak_ut) : DISPENSER_SLOT_NONE;

    /* The peak is the only trustworthy word on which magnet this was, so it is what gets remembered */
    ctx->cal_parked_slot = result.slot;

    err = drv5055_read(ctx->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    result.field_ut = reading.field_ut;

    ESP_LOGI(TAG, "Stopped on slot %d, peak %ld uT, resting %ld uT after %lu ms (%lu passed)", result.slot,
             (long) result.peak_ut, (long) result.field_ut, (unsigned long) result.elapsed_ms,
             (unsigned long) result.magnets_passed);

    if (out_result)
    {
        *out_result = result;
    }

    return ESP_OK;
}

static esp_err_t dispenser_start_motor(dispenser_ctx_t *ctx, dispenser_move_t move)
{
    esp_err_t err = drv8871_set_direction(ctx->motor, dispenser_move_direction(ctx, move));
    if (err != ESP_OK)
    {
        return err;
    }

    return drv8871_set_speed(ctx->motor, ctx->speed_pct);
}

/*
 * Braking pulls the drum up short so it parks close to where the magnet was
 * detected; the brake is released afterwards so the H-bridge is not left
 * holding the winding shorted.
 */
static esp_err_t dispenser_stop_motor(dispenser_ctx_t *ctx, bool brake)
{
    esp_err_t err;

    if (!brake)
    {
        return drv8871_coast(ctx->motor);
    }

    err = drv8871_brake(ctx->motor);
    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(ctx->brake_ms));

    return drv8871_coast(ctx->motor);
}

static int32_t dispenser_magnitude(int32_t field_ut)
{
    return field_ut < 0 ? -field_ut : field_ut;
}

/*
 * Sort ascending. Four elements, so the simplest sort that can carry a second
 * array along with the keys beats anything cleverer.
 */
static void dispenser_sort_pairs(int32_t *keys, int32_t *tags, size_t count)
{
    size_t i;
    size_t j;

    for (i = 1; i < count; i++)
    {
        int32_t key = keys[i];
        int32_t tag = tags ? tags[i] : 0;

        for (j = i; j > 0 && keys[j - 1] > key; j--)
        {
            keys[j] = keys[j - 1];
            if (tags)
            {
                tags[j] = tags[j - 1];
            }
        }

        keys[j] = key;
        if (tags)
        {
            tags[j] = tag;
        }
    }
}

/*
 * Turn until the drum sits between two magnets, then take the quiescent voltage
 * there as the zero-field reference. Zeroing wherever the drum happens to be
 * parked would fold a magnet into the reference and shift every level measured
 * afterwards, which is why this moves first.
 */
static esp_err_t dispenser_zero_between_magnets(dispenser_ctx_t *ctx, const dispenser_cal_config_t *cfg)
{
    esp_err_t         err;
    drv5055_reading_t reading;
    TickType_t        deadline;
    int               zero_mv;

    err = drv5055_read(ctx->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    if (dispenser_magnitude(reading.field_ut) >= cfg->release_ut)
    {
        err = dispenser_start_motor(ctx, cfg->dispensing_direction ? DISPENSER_MOVE_ADVANCE : DISPENSER_MOVE_RETREAT);
        if (err != ESP_OK)
        {
            return err;
        }

        deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);

        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(ctx->poll_interval_ms));

            err = drv5055_read(ctx->hall, &reading);
            if (err != ESP_OK)
            {
                dispenser_stop_motor(ctx, false);
                return err;
            }

            if (dispenser_magnitude(reading.field_ut) < cfg->release_ut)
            {
                break;
            }

            if (xTaskGetTickCount() >= deadline)
            {
                dispenser_stop_motor(ctx, true);
                ESP_LOGE(TAG, "Could not drive clear of a magnet within %lu ms", (unsigned long) ctx->timeout_ms);
                return ESP_ERR_TIMEOUT;
            }
        }

        err = dispenser_stop_motor(ctx, true);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    err = drv5055_calibrate_zero(ctx->hall);
    if (err != ESP_OK)
    {
        return err;
    }

    drv5055_get_zero_millivolts(ctx->hall, &zero_mv);
    ESP_LOGI(TAG, "Zero-field reference captured between magnets at %d mV", zero_mv);

    return ESP_OK;
}

/*
 * Turn the drum without stopping and record the signed peak field of every
 * magnet that goes by. A pass opens when the magnitude crosses the gate and
 * closes when it falls back under the release level, so each magnet contributes
 * exactly one peak however slowly the drum turns. Samples taken between magnets
 * are averaged into a baseline, which costs nothing and says whether the
 * zero-field reference still holds.
 *
 * The timeout is a budget per pass rather than for the whole run, so asking for
 * more revolutions does not have to outrun a single deadline.
 */
static esp_err_t dispenser_capture_peaks(dispenser_ctx_t *ctx, const dispenser_cal_config_t *cfg, int32_t *peaks,
                                         uint32_t want, dispenser_cal_result_t *result)
{
    esp_err_t         err;
    drv5055_reading_t reading;
    int64_t           baseline_sum = 0;
    uint32_t          baseline_n   = 0;
    uint32_t          captured     = 0;
    int32_t           peak_ut      = 0;
    int32_t           magnitude;
    bool              in_pass = false;
    bool              clear_first;
    TickType_t        deadline;

    err = drv5055_read(ctx->hall, &reading);
    if (err != ESP_OK)
    {
        return err;
    }

    /* A drum parked on a magnet would give a peak that was already half over */
    clear_first = dispenser_magnitude(reading.field_ut) >= cfg->release_ut;

    err = dispenser_start_motor(ctx, cfg->dispensing_direction ? DISPENSER_MOVE_ADVANCE : DISPENSER_MOVE_RETREAT);
    if (err != ESP_OK)
    {
        return err;
    }

    if (cfg->speed_pct)
    {
        err = drv8871_set_speed(ctx->motor, cfg->speed_pct);
        if (err != ESP_OK)
        {
            dispenser_stop_motor(ctx, false);
            return err;
        }
    }

    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);

    while (captured < want)
    {
        vTaskDelay(pdMS_TO_TICKS(ctx->poll_interval_ms));

        err = drv5055_read(ctx->hall, &reading);
        if (err != ESP_OK)
        {
            dispenser_stop_motor(ctx, false);
            return err;
        }

        if (reading.saturated)
        {
            dispenser_stop_motor(ctx, true);
            ESP_LOGE(TAG, "The sensor saturated at %ld uT: the magnets sit too close or are too strong to tell apart",
                     (long) reading.field_ut);
            return ESP_ERR_INVALID_STATE;
        }

        magnitude = dispenser_magnitude(reading.field_ut);

        if (clear_first)
        {
            if (magnitude < cfg->release_ut)
            {
                clear_first = false;
                deadline    = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);
            }
        }
        else if (!in_pass)
        {
            if (magnitude >= cfg->gate_ut)
            {
                in_pass = true;
                peak_ut = reading.field_ut;
            }
            else
            {
                baseline_sum += reading.field_ut;
                baseline_n++;
            }
        }
        else
        {
            if (magnitude > dispenser_magnitude(peak_ut))
            {
                peak_ut = reading.field_ut;
            }

            if (magnitude < cfg->release_ut)
            {
                peaks[captured++] = peak_ut;
                in_pass           = false;
                deadline          = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);
            }
        }

        if (xTaskGetTickCount() >= deadline)
        {
            dispenser_stop_motor(ctx, true);
            ESP_LOGE(TAG, "Only %lu of %lu magnet passes within %lu ms, the drum may be jammed",
                     (unsigned long) captured, (unsigned long) want, (unsigned long) ctx->timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }

    err = dispenser_stop_motor(ctx, true);
    if (err != ESP_OK)
    {
        return err;
    }

    result->peaks_seen  = captured;
    result->baseline_ut = baseline_n ? (int32_t) (baseline_sum / (int64_t) baseline_n) : 0;

    if (dispenser_magnitude(result->baseline_ut) > DISPENSER_CAL_BASELINE_WARN_UT)
    {
        ESP_LOGW(TAG,
                 "The field between magnets averaged %ld uT rather than nothing, so the zero-field reference is "
                 "off; recalibrate it or pass recalibrate_zero",
                 (long) result->baseline_ut);
    }

    return ESP_OK;
}

/*
 * Turn a run of peaks into a level per slot.
 *
 * Peaks arrive in the order the magnets went by, starting from wherever the drum
 * happened to be, so peak i and peak i + DISPENSER_SLOT_COUNT are the same
 * magnet one revolution later and average together. That gives four levels in
 * drum order but with an unknown starting slot, and in reverse when the run
 * turned the non-dispensing way.
 *
 * Anchoring them to real slots uses the mounting: slots 0 and 1 share one height
 * and slots 2 and 3 the other, so the levels read as two of one strength
 * followed by two of the other, and a cyclic run of two-and-two lines up with
 * the drum exactly one way once you know which pair slots 0 and 1 are in.
 */
static esp_err_t dispenser_derive_map(dispenser_ctx_t *ctx, const dispenser_cal_config_t *cfg, const int32_t *peaks,
                                      uint32_t count, dispenser_cal_result_t *result)
{
    int32_t  level[DISPENSER_SLOT_COUNT];
    int32_t  spread[DISPENSER_SLOT_COUNT];
    int32_t  magnitude[DISPENSER_SLOT_COUNT];
    int32_t  by_magnitude[DISPENSER_SLOT_COUNT];
    bool     strong[DISPENSER_SLOT_COUNT];
    uint32_t per_slot       = count / DISPENSER_SLOT_COUNT;
    int32_t  worst_spread   = 0;
    int32_t  tier_gap       = 0;
    int32_t  split          = 0;
    int      offset         = -1;
    int      matches        = 0;
    bool     home_is_strong = !cfg->home_pair_is_weaker;
    size_t   i;
    size_t   j;

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        int64_t sum     = 0;
        int32_t lowest  = peaks[i];
        int32_t highest = peaks[i];

        for (j = 0; j < per_slot; j++)
        {
            int32_t value = peaks[i + j * DISPENSER_SLOT_COUNT];

            sum += value;
            lowest  = value < lowest ? value : lowest;
            highest = value > highest ? value : highest;
        }

        level[i]  = (int32_t) (sum / (int64_t) per_slot);
        spread[i] = highest - lowest;

        worst_spread = spread[i] > worst_spread ? spread[i] : worst_spread;
    }

    /* Turning the non-dispensing way visits the slots backwards */
    if (!cfg->dispensing_direction)
    {
        for (i = 0; i < DISPENSER_SLOT_COUNT / 2; i++)
        {
            size_t  mirror = DISPENSER_SLOT_COUNT - 1 - i;
            int32_t swap   = level[i];

            level[i]      = level[mirror];
            level[mirror] = swap;

            swap           = spread[i];
            spread[i]      = spread[mirror];
            spread[mirror] = swap;
        }
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        magnitude[i]    = dispenser_magnitude(level[i]);
        by_magnitude[i] = magnitude[i];
    }

    /* The two mounting heights show up as two clusters of two; split at the median */
    dispenser_sort_pairs(by_magnitude, NULL, DISPENSER_SLOT_COUNT);

    tier_gap = by_magnitude[2] - by_magnitude[1];
    split    = by_magnitude[1] + tier_gap / 2;

    if (tier_gap < DISPENSER_CAL_MIN_TIER_GAP_UT || tier_gap <= 2 * worst_spread)
    {
        ESP_LOGE(TAG,
                 "The two mounting heights are only %ld uT apart with %ld uT of scatter; increase the height step "
                 "or check the shims",
                 (long) tier_gap, (long) worst_spread);
        return ESP_ERR_INVALID_STATE;
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        strong[i] = magnitude[i] >= split;
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        if (strong[i] == home_is_strong && strong[(i + 1) % DISPENSER_SLOT_COUNT] == home_is_strong &&
            strong[(i + 2) % DISPENSER_SLOT_COUNT] != home_is_strong &&
            strong[(i + 3) % DISPENSER_SLOT_COUNT] != home_is_strong)
        {
            offset = (int) i;
            matches++;
        }
    }

    if (matches != 1)
    {
        ESP_LOGE(TAG,
                 "Read %s%s%s%s around the drum, which is not two of one height then two of the other; check which "
                 "pockets were shimmed",
                 strong[0] ? "S" : "W", strong[1] ? "S" : "W", strong[2] ? "S" : "W", strong[3] ? "S" : "W");
        return ESP_ERR_NOT_FOUND;
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        size_t slot = (size_t) (((int) i - offset + DISPENSER_SLOT_COUNT) % DISPENSER_SLOT_COUNT);

        result->level_ut[slot]  = level[i];
        result->spread_ut[slot] = spread[i];
    }

    return dispenser_activate_map(ctx, result->level_ut, 0, 0, result);
}

/*
 * Put a set of levels to work: order them, drop a decision boundary midway
 * between each neighbouring pair and refuse the lot if any two are too close to
 * tell apart. Two magnets at the same height with the same pole facing the
 * sensor land on top of each other here, which is the check that catches one
 * inserted the wrong way round.
 *
 * A gate_ut of 0 asks for the gate to be derived as half the weakest magnitude,
 * which sits well clear of both nothing and the weakest magnet; any other value
 * is taken as given, which is how a stored map keeps the gate it was calibrated
 * with. The gate is pushed into the Hall driver so that its presence flag and
 * this map agree on where a magnet starts, and a zero_mv of 0 leaves the
 * existing reference alone.
 */
static esp_err_t dispenser_activate_map(dispenser_ctx_t *ctx, const int32_t *level_ut, int32_t gate_ut, int32_t zero_mv,
                                        dispenser_cal_result_t *result)
{
    int32_t   ordered_level[DISPENSER_SLOT_COUNT];
    int32_t   ordered_slot[DISPENSER_SLOT_COUNT];
    int32_t   smallest_gap = INT32_MAX;
    esp_err_t err;
    size_t    i;

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        ordered_level[i] = level_ut[i];
        ordered_slot[i]  = (int32_t) i;
    }

    dispenser_sort_pairs(ordered_level, ordered_slot, DISPENSER_SLOT_COUNT);

    for (i = 0; i < DISPENSER_SLOT_COUNT - 1; i++)
    {
        int32_t gap = ordered_level[i + 1] - ordered_level[i];

        if (gap < DISPENSER_CAL_MIN_SEPARATION_UT)
        {
            ESP_LOGE(TAG,
                     "Slots %ld and %ld read only %ld uT apart, under the %d uT needed to tell them apart; they are "
                     "probably at the same height with the same pole facing the sensor",
                     (long) ordered_slot[i], (long) ordered_slot[i + 1], (long) gap, DISPENSER_CAL_MIN_SEPARATION_UT);
            return ESP_ERR_INVALID_STATE;
        }

        smallest_gap = gap < smallest_gap ? gap : smallest_gap;
    }

    if (gate_ut <= 0)
    {
        gate_ut = dispenser_magnitude(ordered_level[0]);

        for (i = 1; i < DISPENSER_SLOT_COUNT; i++)
        {
            int32_t candidate = dispenser_magnitude(ordered_level[i]);

            gate_ut = candidate < gate_ut ? candidate : gate_ut;
        }

        gate_ut /= 2;
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        ctx->cal_level_ut[i]     = level_ut[i];
        ctx->cal_ordered_ut[i]   = ordered_level[i];
        ctx->cal_ordered_slot[i] = (int) ordered_slot[i];
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT - 1; i++)
    {
        ctx->cal_boundary_ut[i] = ordered_level[i] + (ordered_level[i + 1] - ordered_level[i]) / 2;
    }

    ctx->cal_margin_ut = smallest_gap / 2;
    ctx->cal_gate_ut   = gate_ut;

    if (zero_mv)
    {
        err = drv5055_set_zero_millivolts(ctx->hall, zero_mv);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    err = drv5055_get_zero_millivolts(ctx->hall, &ctx->cal_zero_mv);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Keep the presence flag the driver latches in step with the gate of this map */
    err = drv5055_set_threshold(ctx->hall, gate_ut, gate_ut / 4);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Levels have moved, so a slot number remembered against the old ones means nothing */
    ctx->cal_parked_slot = DISPENSER_SLOT_NONE;
    ctx->cal_valid       = true;

    if (result)
    {
        for (i = 0; i < DISPENSER_SLOT_COUNT - 1; i++)
        {
            result->boundary_ut[i] = ctx->cal_boundary_ut[i];
        }

        result->margin_ut = ctx->cal_margin_ut;
        result->gate_ut   = ctx->cal_gate_ut;
        result->zero_mv   = ctx->cal_zero_mv;
    }

    return ESP_OK;
}

static esp_err_t dispenser_save_map(dispenser_ctx_t *ctx)
{
    dispenser_stored_map_t stored = {0};
    nvs_handle_t           nvs;
    esp_err_t              err;
    size_t                 i;

    stored.magic      = DISPENSER_MAP_MAGIC;
    stored.version    = DISPENSER_MAP_VERSION;
    stored.slot_count = DISPENSER_SLOT_COUNT;
    stored.zero_mv    = ctx->cal_zero_mv;
    stored.gate_ut    = ctx->cal_gate_ut;

    for (i = 0; i < DISPENSER_SLOT_COUNT; i++)
    {
        stored.level_ut[i] = ctx->cal_level_ut[i];
    }

    err = nvs_open(DISPENSER_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_set_blob(nvs, DISPENSER_NVS_KEY_MAP, &stored, sizeof(stored));
    if (err == ESP_OK)
    {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);
    return err;
}

/*
 * The gate is stored as it was applied rather than recomputed, so a reboot puts
 * back exactly the one the drum was calibrated with.
 */
static esp_err_t dispenser_load_map(dispenser_ctx_t *ctx)
{
    dispenser_stored_map_t stored;
    nvs_handle_t           nvs;
    size_t                 length = sizeof(stored);
    esp_err_t              err;

    err = nvs_open(DISPENSER_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_get_blob(nvs, DISPENSER_NVS_KEY_MAP, &stored, &length);
    nvs_close(nvs);

    if (err != ESP_OK)
    {
        return err;
    }

    if (length != sizeof(stored) || stored.magic != DISPENSER_MAP_MAGIC || stored.version != DISPENSER_MAP_VERSION ||
        stored.slot_count != DISPENSER_SLOT_COUNT)
    {
        ESP_LOGW(TAG, "Ignoring a stored slot map this build cannot read");
        return ESP_ERR_INVALID_VERSION;
    }

    return dispenser_activate_map(ctx, stored.level_ut, stored.gate_ut, stored.zero_mv, NULL);
}

/*
 * Which slot the drum is showing, from one signed field reading. Below the gate
 * no magnet is in front of the sensor; above it the reading falls into one of
 * four bands, and the sign is what separates the two magnets sharing a height.
 */
/*
 * The fall-back from the peak that ends a magnet pass, never less than the floor
 * so that a noisy sample near a weak magnet's crest cannot end it early.
 */
static int32_t dispenser_peak_drop(int32_t peak_ut)
{
    int32_t drop = dispenser_magnitude(peak_ut) * DISPENSER_PEAK_DROP_PERCENT / 100;

    return drop > DISPENSER_PEAK_DROP_MIN_UT ? drop : DISPENSER_PEAK_DROP_MIN_UT;
}

/*
 * The slot the drum is standing on, as far as anything knows.
 *
 * This is the remembered outcome of the last move rather than a reading of the
 * field as it stands, because a move parks the drum past the magnet's peak: the
 * field there has fallen off enough that classifying it can name a weaker slot
 * than the magnet actually under the sensor. The reading is only used to notice
 * that the remembered answer has gone stale, which it has if the drum no longer
 * shows that slot's pole or has hardly any field on it at all.
 */
static int dispenser_parked_slot(const dispenser_ctx_t *ctx, int32_t field_ut)
{
    int32_t level;

    if (!ctx->cal_valid || ctx->cal_parked_slot == DISPENSER_SLOT_NONE)
    {
        return DISPENSER_SLOT_NONE;
    }

    if (dispenser_magnitude(field_ut) < DISPENSER_PARKED_MIN_UT)
    {
        return DISPENSER_SLOT_NONE;
    }

    level = ctx->cal_level_ut[ctx->cal_parked_slot];

    /* Only the pole survives the fall-off past the peak; the magnitude says nothing useful */
    if ((level < 0) != (field_ut < 0))
    {
        return DISPENSER_SLOT_NONE;
    }

    return ctx->cal_parked_slot;
}

/*
 * Whether a move has nothing to do because the drum is already where it was
 * asked to go. An uncalibrated drum has no remembered position to go on, so it
 * falls back to reading the field, which can still pick out home by its pole.
 */
static bool dispenser_already_at(const dispenser_ctx_t *ctx, int target_slot, const drv5055_reading_t *reading)
{
    if (target_slot == DISPENSER_SLOT_NONE)
    {
        return false;
    }

    if (ctx->cal_valid)
    {
        return dispenser_parked_slot(ctx, reading->field_ut) == target_slot;
    }

    return reading->magnet_present && dispenser_magnet_matches(ctx, target_slot, reading->field_ut);
}

/*
 * Turn until the drum is clear of whatever magnet it is on, then keep going for
 * extra_ms to put some daylight between the two, and stop.
 *
 * The release side of the Hall driver's hysteresis decides what counts as clear,
 * the same test a move uses to tell that it has driven off the magnet it started
 * on, so the two agree about where a magnet ends.
 */
static esp_err_t dispenser_drive_clear(dispenser_ctx_t *ctx, dispenser_move_t move, uint32_t extra_ms)
{
    esp_err_t         err;
    drv5055_reading_t reading;
    TickType_t        deadline;

    err = dispenser_start_motor(ctx, move);
    if (err != ESP_OK)
    {
        return err;
    }

    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ctx->timeout_ms);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ctx->poll_interval_ms));

        err = drv5055_read(ctx->hall, &reading);
        if (err != ESP_OK)
        {
            dispenser_stop_motor(ctx, false);
            return err;
        }

        if (!reading.magnet_present && !dispenser_within_magnet(ctx, reading.field_ut))
        {
            break;
        }

        if (xTaskGetTickCount() >= deadline)
        {
            dispenser_stop_motor(ctx, true);
            ctx->cal_parked_slot = DISPENSER_SLOT_NONE;
            ESP_LOGE(TAG, "Could not drive clear of a magnet within %lu ms", (unsigned long) ctx->timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }

    if (extra_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(extra_ms));
    }

    return dispenser_stop_motor(ctx, true);
}

static int dispenser_slot_for_field(const dispenser_ctx_t *ctx, int32_t field_ut)
{
    size_t i;

    if (dispenser_magnitude(field_ut) < ctx->cal_gate_ut)
    {
        return DISPENSER_SLOT_NONE;
    }

    for (i = 0; i < DISPENSER_SLOT_COUNT - 1; i++)
    {
        if (field_ut < ctx->cal_boundary_ut[i])
        {
            return ctx->cal_ordered_slot[i];
        }
    }

    return ctx->cal_ordered_slot[DISPENSER_SLOT_COUNT - 1];
}
