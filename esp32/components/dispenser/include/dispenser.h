#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drv5055.h"
#include "drv8871.h"
#include "esp_err.h"
#include "max98357a.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct dispenser_t *dispenser_handle_t;

    /* Sound played once a move finishes; a repeat_count of 0 stays silent */
    typedef struct
    {
        max98357a_melody_t melody;
        uint32_t           repeat_count;
        uint32_t           volume_pct;
    } dispenser_chime_t;

    /* Which magnet a move hunts for, and so which way the drum has to turn */
    typedef enum
    {
        DISPENSER_MOVE_HOME = 0, /* the home magnet, turning the dispensing way */
        DISPENSER_MOVE_ADVANCE,  /* the next magnet, dispensing a treat */
        DISPENSER_MOVE_RETREAT,  /* the previous magnet, turning back and dispensing nothing */
    } dispenser_move_t;

/*
 * Slots in the drum, numbered the way the drum turns when dispensing. Each slot
 * carries one magnet; the pair on slots 0 and 1 sits at one height and the pair
 * on slots 2 and 3 at another, and within each pair the two magnets face the
 * sensor with opposite poles. That gives four distinct signed field levels, so a
 * single Hall reading says which slot is at the opening.
 */
#define DISPENSER_SLOT_COUNT 4
#define DISPENSER_SLOT_HOME  0

    /* Returned by dispenser_get_slot() when the drum is parked between magnets */
#define DISPENSER_SLOT_NONE (-1)

    typedef struct
    {
        drv8871_handle_t   motor_handle; /* initialized DRV8871 handle */
        drv5055_handle_t   hall_handle;  /* initialized DRV5055 handle */
        max98357a_handle_t audio_handle; /* initialized MAX98357A handle, NULL to stay silent */

        dispenser_chime_t home_chime;    /* played when dispenser_home() reaches the home magnet */
        dispenser_chime_t advance_chime; /* played when dispenser_advance() reaches the next magnet */
        dispenser_chime_t retreat_chime; /* played when dispenser_retreat() reaches the previous magnet */

        drv8871_direction_t direction;        /* direction the drum turns while dispensing */
        uint32_t            travel_speed_pct; /* motor speed used for moves, 0 selects 60 % */
        uint32_t            poll_interval_ms; /* Hall sampling interval, 0 selects 10 ms */
        uint32_t            timeout_ms;       /* budget per magnet-to-magnet hop, 0 selects 5000 ms */
        uint32_t            brake_ms;         /* active braking once a magnet is found, 0 selects 150 ms */
    } dispenser_config_t;

    typedef struct
    {
        int32_t  field_ut;       /* field measured after the drum came to a stop */
        int32_t  peak_ut;        /* strongest field seen crossing the magnet, what the slot is judged on */
        int      slot;           /* slot the move landed on, DISPENSER_SLOT_NONE when no map is active */
        uint32_t magnets_passed; /* magnets skipped before the target one was reached */
        uint32_t elapsed_ms;     /* time the motor ran */
        bool     already_there;  /* the drum was already parked on the target magnet, it did not move */
    } dispenser_result_t;

    /**
     * Initialize the dispenser motion layer.
     *
     * Pairs the H-bridge with the Hall sensor so the drum can be moved from one
     * magnet to the next. The motor is not touched here; it keeps whatever state
     * drv8871_init() left it in.
     *
     * @param config Motion configuration.
     * @param out_handle Receives the handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle or a
     *         speed above 100, ESP_ERR_NO_MEM when the handle cannot be allocated.
     */
    esp_err_t dispenser_init(const dispenser_config_t *config, dispenser_handle_t *out_handle);

    /**
     * Turn the drum until it parks on the home magnet.
     *
     * Home is slot 0. With a slot map active it is recognised by its field level
     * like any other slot, and the magnets of the other three are counted and
     * driven past; without one, homing falls back to the older arrangement where
     * home was the only magnet turned round and so the only one reading
     * negative. Returns immediately when the drum is already parked on home.
     *
     * A magnet is judged by the strongest field seen while crossing it, not by
     * the field at the moment it first trips the threshold, so a slot is
     * identified from the peak whatever the drum does afterwards.
     *
     * Blocks until the magnet is found or the move times out, and always leaves
     * the motor stopped. On success the configured home chime starts playing in
     * the background, including when the drum was already there.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT when no magnet showed up within
     *         the configured budget, ESP_ERR_NOT_FOUND when a full drum of
     *         magnets went by without the home one, or an error from the motor or
     *         Hall driver.
     */
    esp_err_t dispenser_home(dispenser_handle_t handle, dispenser_result_t *out_result);

    /**
     * Turn the drum on to the next magnet, whatever its polarity.
     *
     * Always moves: a drum that is already parked on a magnet first drives clear
     * of it, so this advances exactly one position per call.
     *
     * Blocks until the magnet is found or the move times out, and always leaves
     * the motor stopped. On success the configured advance chime starts playing
     * in the background.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT when the next magnet did not
     *         arrive within the configured budget, or an error from the motor or
     *         Hall driver.
     */
    esp_err_t dispenser_advance(dispenser_handle_t handle, dispenser_result_t *out_result);

    /**
     * Turn the drum back to the previous magnet, whatever its polarity.
     *
     * The mirror image of dispenser_advance(): the drum turns the opposite way
     * and gives nothing out, which is what undoes a slot advanced by mistake.
     * Always moves, exactly one position per call.
     *
     * Blocks until the magnet is found or the move times out, and always leaves
     * the motor stopped. On success the configured retreat chime starts playing
     * in the background.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT when the previous magnet did
     *         not arrive within the configured budget, or an error from the
     *         motor or Hall driver.
     */
    esp_err_t dispenser_retreat(dispenser_handle_t handle, dispenser_result_t *out_result);

    /**
     * Make any of the three moves, with a chime of the caller's choosing.
     *
     * dispenser_home(), dispenser_advance() and dispenser_retreat() are this
     * with the configured chime. Passing one here overrides that, which is how
     * a caller that acknowledges a move some other way — the phone link showing
     * it on screen, say — keeps the drum from bursting into song.
     *
     * @param handle Dispenser handle.
     * @param move Which move to make.
     * @param chime Chime to play on success, NULL for the configured one. A
     *              chime with a repeat_count of 0 stays silent.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for an unknown move, or the
     *         errors the individual moves report.
     */
    esp_err_t dispenser_move(dispenser_handle_t handle, dispenser_move_t move, const dispenser_chime_t *chime,
                             dispenser_result_t *out_result);

    /**
     * Turn the drum until the wanted slot is at the opening.
     *
     * The slot map makes position absolute, so any slot can be asked for by
     * number rather than counted up to: the drum turns until the one wanted is
     * at the opening, driving past the slots in between. Slot 0 is home, so
     * asking for it does the same job as dispenser_home().
     *
     * It always turns the dispensing way, never taking a shorter way round
     * backwards, because a move stops just past the peak of the magnet it
     * settles on. Coming at every slot from the same side puts that overshoot on
     * the same side too, and takes up the gear backlash the same way, so a slot
     * lands in the same place every time rather than a step ahead or behind
     * depending on where the drum came from.
     *
     * The cost is that reaching a slot can mean going most of the way round, and
     * every pocket crossed passes the opening on the dispensing side, so this
     * gives out treats exactly as dispenser_advance() does — up to
     * DISPENSER_SLOT_COUNT - 1 of them.
     *
     * Blocks until the slot is reached or the move times out, and always leaves
     * the motor stopped. Returns immediately when the drum is already parked on
     * that slot, which is the one case where nothing is given out.
     *
     * @param handle Dispenser handle.
     * @param slot Slot to park at, 0 to DISPENSER_SLOT_COUNT - 1.
     * @param chime Chime to play on success, NULL for the configured advance
     *              chime. A chime with a repeat_count of 0 stays silent.
     * @param out_result Receives details of the move, may be NULL.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a slot out of range,
     *         ESP_ERR_INVALID_STATE when no slot map is active so slots cannot be
     *         told apart, ESP_ERR_TIMEOUT when a magnet did not arrive within the
     *         budget, ESP_ERR_NOT_FOUND when a full drum went by without that
     *         slot, or an error from the motor or Hall driver.
     */
    esp_err_t dispenser_go_to_slot(dispenser_handle_t handle, int slot, const dispenser_chime_t *chime,
                                   dispenser_result_t *out_result);

    /**
     * Read whether the drum is currently parked on the home magnet.
     *
     * @param handle Dispenser handle.
     * @param out_home Receives the result.
     * @return ESP_OK on success or an error from the Hall driver.
     */
    esp_err_t dispenser_is_home(dispenser_handle_t handle, bool *out_home);

    /**
     * Called as a move begins and again once it has finished.
     *
     * The second call comes after the chime has been started rather than after
     * it has finished, playback running in its own task; an observer that wants
     * to wait for the sound can watch max98357a_is_playing() from there.
     *
     * Runs in whichever task asked for the move, with no dispenser lock held.
     */
    typedef void (*dispenser_move_cb_t)(bool moving, void *ctx);

    /**
     * Watch the drum turn.
     *
     * One observer, called around every move that dispenser_move() and
     * dispenser_go_to_slot() make, which is every move anything asks for: the
     * schedule, the phone and the console all arrive through those two.
     *
     * Deliberately not called for dispenser_find_position() or
     * dispenser_calibrate(). Those turn the drum too, but they are the dispenser
     * seeing to itself rather than giving anything out, and neither plays a
     * chime for an observer to wait on.
     *
     * @param handle Dispenser handle.
     * @param callback Observer, or NULL to stop watching.
     * @param ctx Passed back to the callback.
     * @return ESP_OK on success.
     */
    esp_err_t dispenser_set_move_observer(dispenser_handle_t handle, dispenser_move_cb_t callback, void *ctx);

    /**
     * Set the motor speed used for moves, in percent.
     *
     * @param handle Dispenser handle.
     * @param speed_pct Speed in percent (1-100).
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out of range values.
     */
    esp_err_t dispenser_set_travel_speed(dispenser_handle_t handle, uint32_t speed_pct);

    /**
     * Get the motor speed used for moves, in percent.
     */
    esp_err_t dispenser_get_travel_speed(dispenser_handle_t handle, uint32_t *out_speed_pct);

    typedef struct
    {
        uint32_t revolutions; /* full turns averaged over, 0 selects 3 */
        uint32_t speed_pct;   /* motor speed while turning, 0 selects the configured travel speed */

        int32_t gate_ut;    /* field magnitude that opens a magnet pass, 0 selects 3000 uT */
        int32_t release_ut; /* magnitude that closes it again, 0 selects half the gate */

        bool dispensing_direction; /* turn the dispensing way; the default turns back so no treats fall out */
        bool home_pair_is_weaker;  /* slots 0 and 1 read weaker; the default is that they read stronger */
        bool recalibrate_zero;     /* capture a fresh zero-field reference before turning */
        bool skip_save;            /* leave non-volatile storage alone, just report what was measured */
    } dispenser_cal_config_t;

    typedef struct
    {
        int32_t level_ut[DISPENSER_SLOT_COUNT];  /* averaged signed peak, indexed by slot */
        int32_t spread_ut[DISPENSER_SLOT_COUNT]; /* peak-to-peak scatter across revolutions, per slot */

        int32_t boundary_ut[DISPENSER_SLOT_COUNT - 1]; /* decision boundaries, ascending */
        int32_t margin_ut;                             /* smallest gap between a level and its boundary */

        int32_t  gate_ut;     /* magnitude below which no magnet is in front of the sensor */
        int32_t  zero_mv;     /* zero-field reference the levels were measured against */
        uint32_t peaks_seen;  /* magnet passes captured */
        int32_t  baseline_ut; /* mean field seen between magnets, a check on the zero reference */
        bool     saved;       /* the map was written to non-volatile storage */
    } dispenser_cal_result_t;

    /**
     * Learn the field level of every slot and store the resulting map.
     *
     * Turns the drum continuously for the configured number of revolutions,
     * recording the signed peak field of each magnet as it goes by, then averages
     * the passes belonging to each slot and places a decision boundary midway
     * between neighbouring levels. On success the map is written to non-volatile
     * storage and becomes active straight away, so a later boot only has to call
     * dispenser_calibration_load().
     *
     * Which measured level belongs to which slot follows from the mounting: the
     * two magnets sharing a height are on adjacent slots, so the run of two weak
     * levels followed by two strong ones can only line up with the drum one way.
     * home_pair_is_weaker says which of the two heights slots 0 and 1 are at, and
     * is the one thing the routine cannot work out for itself. It defaults to
     * false, meaning slots 0 and 1 read stronger, which is what the Hall sensor
     * sitting below the tray gives: slots 2 and 3 are the raised pair, so
     * raising them moves them away from the sensor and weakens them.
     *
     * By default the drum turns the non-dispensing way so nothing falls out, but
     * run it on an empty drum regardless: several full revolutions go past the
     * opening.
     *
     * Blocks for the whole run and always leaves the motor stopped. Plays no
     * chime.
     *
     * @param handle Dispenser handle.
     * @param config Calibration settings, NULL for all defaults.
     * @param out_result Receives the measured map, may be NULL.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a bad setting,
     *         ESP_ERR_TIMEOUT when a magnet did not arrive in time,
     *         ESP_ERR_INVALID_RESPONSE when the captured passes do not add up to
     *         a whole number of revolutions, ESP_ERR_NOT_FOUND when the two
     *         magnets at the same height are not on adjacent slots,
     *         ESP_ERR_INVALID_STATE when two levels are too close to tell apart,
     *         or an error from the motor, Hall or NVS driver.
     */
    esp_err_t dispenser_calibrate(dispenser_handle_t handle, const dispenser_cal_config_t *config,
                                  dispenser_cal_result_t *out_result);

    /**
     * Load the stored slot map and make it active.
     *
     * Called by dispenser_init(), so the map from the last calibration is already
     * in place; call it again only to undo an uncommitted dispenser_calibrate()
     * run. Also restores the zero-field reference the map was measured against,
     * because the levels mean nothing without it.
     *
     * @param handle Dispenser handle.
     * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND when nothing has been
     *         stored yet, ESP_ERR_INVALID_VERSION when the stored map came from
     *         an incompatible build, or an error from the NVS driver.
     */
    esp_err_t dispenser_calibration_load(dispenser_handle_t handle);

    /**
     * Read back the active slot map without turning the drum.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives the map. spread_ut and peaks_seen are zero
     *                   unless this handle ran the calibration itself.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when no map is active.
     */
    esp_err_t dispenser_calibration_get(dispenser_handle_t handle, dispenser_cal_result_t *out_result);

    /**
     * Forget the stored slot map, on this handle and in non-volatile storage.
     *
     * Homing falls back to treating the single negative magnet as home, which is
     * the behaviour from before the slots were told apart by height.
     *
     * @param handle Dispenser handle.
     * @return ESP_OK on success or an error from the NVS driver.
     */
    esp_err_t dispenser_calibration_erase(dispenser_handle_t handle);

    /**
     * Read whether a slot map is active.
     */
    bool dispenser_is_calibrated(dispenser_handle_t handle);

    /**
     * Find out which slot the drum is standing on, without sending it anywhere.
     *
     * Nothing says where a freshly powered drum is standing, and the field as it
     * reads cannot be trusted to say either: the move that parked it stopped
     * past the magnet's peak, so the field there has fallen off enough to look
     * like a weaker slot. The way to find out is to measure a peak, and the way
     * to measure a peak is to cross one.
     *
     * So the drum backs off the magnet it is on, far enough for the field to
     * fall away, then turns forward again over the same magnet and stops on its
     * peak. That names the slot, and it leaves the drum approached from the
     * dispensing side exactly as dispenser_go_to_slot() would, so the position
     * it settles at matches every later move rather than being wherever the last
     * session happened to leave it.
     *
     * It stays on the slot it started on, backing off well under half a slot, so
     * nothing crosses the opening and nothing is given out. The exception is a
     * drum that starts parked between magnets, having been turned by hand: there
     * is no magnet to back off, so the forward sweep runs on to the next slot
     * and can give out what it crosses.
     *
     * Blocks for the two short moves and always leaves the motor stopped. Plays
     * no chime, this being something the dispenser does to itself at start-up
     * rather than a move anybody asked for.
     *
     * @param handle Dispenser handle.
     * @param out_result Receives details of the forward sweep, may be NULL.
     *                   already_there is set, and neither move made, when the
     *                   position was known already.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when no slot map is
     *         active so slots cannot be told apart, ESP_ERR_TIMEOUT when the drum
     *         did not move as expected, or an error from the motor or Hall
     *         driver.
     */
    esp_err_t dispenser_find_position(dispenser_handle_t handle, dispenser_result_t *out_result);

    /**
     * Read which slot is at the opening.
     *
     * Reports the slot the last move settled on, which it judged from the peak
     * field measured while the magnet was centred on the sensor. It is not a
     * fresh classification of the field as it reads now: a move stops a little
     * past the peak, so the field at rest has already fallen away and can sit in
     * a weaker slot's band than the magnet the drum is actually on. Judging
     * position from that reading names the wrong slot, which is why the answer
     * comes from the move instead.
     *
     * The reading is still taken, as a check that the remembered slot has not
     * gone stale: the drum has to still show the same pole with some field
     * behind it. Turned away from its magnet by hand it does not, and the answer
     * is then DISPENSER_SLOT_NONE rather than a slot the drum has left.
     *
     * Position is therefore unknown until the drum has been moved once, since
     * nothing says where a drum that has only just been powered up is standing.
     * Any move establishes it, dispenser_home() included.
     *
     * @param handle Dispenser handle.
     * @param out_slot Receives the slot number, or DISPENSER_SLOT_NONE when the
     *                 drum has not been moved yet, was moved by hand since, or
     *                 is parked between magnets.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when no map is active, or
     *         an error from the Hall driver.
     */
    esp_err_t dispenser_get_slot(dispenser_handle_t handle, int *out_slot);

    /**
     * Match an already-measured field against the map.
     *
     * The arithmetic behind dispenser_get_slot(), split out so a caller that
     * already has a reading — the field a dispenser_result_t reports, say — does
     * not have to take another one.
     *
     * @param handle Dispenser handle.
     * @param field_ut Signed flux density in microtesla.
     * @param out_slot Receives the slot number, or DISPENSER_SLOT_NONE.
     * @return ESP_OK on success or ESP_ERR_INVALID_STATE when no map is active.
     */
    esp_err_t dispenser_classify_field(dispenser_handle_t handle, int32_t field_ut, int *out_slot);

#ifdef __cplusplus
}
#endif
