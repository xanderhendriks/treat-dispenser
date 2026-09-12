#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ble_remote.h"
#include "esp_err.h"
#include "gc9a01a.h"
#include "max98357a.h"
#include "rv3028.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * The dispenser's face: the wall clock in small digits near the top, and how
     * long until the next treat in large ones across the middle. Both are seven
     * segment digits drawn out of rectangles, white on black, with no frame,
     * border or divider anywhere.
     *
     * That last part is deliberate rather than minimalism for its own sake. The
     * panel datasheet's advice for anything that stays on screen for hours is to
     * avoid distinct lines as borders, keep large static areas dark or mid grey,
     * and let nothing sit unchanged for long. A black field with thin strokes
     * that tick over every second is about as close to that as a useful display
     * gets.
     *
     * What is left is that the strokes, however often their values change, keep
     * landing on the same pixels; the two colons never change at all. So the
     * whole face is moved about the panel, a step at a time round an eight point
     * ring, and the digits are sized to leave room for that rather than to fill
     * the glass. The step is wider than a stroke, which is the part that
     * matters: a shift of one or two pixels leaves most of every segment
     * standing where it already was.
     *
     * A redraw is not reported to the screen care layer as activity. If it were,
     * a countdown ticking once a second would reset the idle clock once a second
     * and the panel would never dim or nudge. The clock the care layer watches is
     * therefore a clock of real events, and the screen only pushes it along when
     * the schedule moves on, which is to say just after a treat has been
     * dispensed.
     */

    /** Drift interval that pins the face in one place. */
#define SCREEN_DRIFT_NEVER UINT32_MAX

    /** Ceiling on the search for a usable drift radius; the glass is the real limit. */
#define SCREEN_DRIFT_MAX_RADIUS_PX 32

    /**
     * Hold the dog up until something ends it, rather than for a set time.
     *
     * For the boot sequence, where what he is covering is the drum going looking
     * for itself and nobody knows in advance how long that takes.
     */
#define SCREEN_CELEBRATE_HOLD UINT32_MAX

    /**
     * Hold the dog up until the dispenser has finished making noise.
     *
     * For a move: the drum turns, the chime plays, and the dog stays up over
     * both, coming off when the last note does. Ends on the next tick when
     * there was no chime to wait for.
     */
#define SCREEN_CELEBRATE_UNTIL_QUIET (UINT32_MAX - 1)

    typedef struct screen_t *screen_handle_t;

    typedef struct
    {
        gc9a01a_handle_t display_handle; /* initialized panel handle */

        /*
         * Both may be NULL, and the face degrades rather than refusing to
         * start: without an RTC the clock shows dashes, and without a schedule
         * so does the countdown. Dashes also stand in while the RTC is
         * reporting that its time was never set.
         */
        rv3028_handle_t    rtc_handle;
        scheduler_handle_t schedule_handle;

        /*
         * Also optional, and what the Bluetooth icon beside the dog is driven
         * from: it pulses while the pairing window is open, sits dim once it has
         * shut, and is not drawn at all when the radio never came up. An icon
         * that is there whether or not the thing works would say nothing.
         */
        ble_remote_handle_t ble_handle;

        /*
         * Optional too, and what SCREEN_CELEBRATE_UNTIL_QUIET waits on. Without
         * it that degrades to ending on the next tick.
         */
        max98357a_handle_t audio_handle;

        /*
         * How often the face moves to a new place on the panel, and how far
         * from the middle it is allowed to get. 0 selects a step every minute,
         * and a radius of 0 selects however much slack the layout leaves
         * between itself and the glass, which is the sensible default: adding
         * something to the face then costs travel rather than needing this
         * re-tuned. SCREEN_DRIFT_NEVER pins the face in place.
         *
         * An explicit radius is clamped to what fits, so it can be asked for
         * freely and will simply stop growing.
         */
        uint32_t drift_interval_ms;
        uint8_t  drift_radius_px;

        /*
         * Open on the dog rather than on the face, and hold him there until
         * screen_celebrate(handle, 0) says otherwise. Set before the first frame
         * rather than asked for afterwards, so there is no moment where the
         * clock is on the glass first and the backlight comes up on the wrong
         * thing.
         */
        bool splash;

        uint32_t task_stack_size; /* 0 selects 3072 bytes */
        uint32_t task_priority;   /* 0 selects priority 2 */
    } screen_config_t;

    /**
     * Start drawing the dispenser's face.
     *
     * Paints the layout once and then starts a task that keeps it current,
     * repainting only the digits that actually changed. The panel's backlight is
     * left exactly as it was, so a caller that wants the first frame up before
     * anything is lit can order it that way.
     *
     * @param config Screen configuration.
     * @param out_handle Receives the handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG without a display,
     *         ESP_ERR_NO_MEM when the task or handle cannot be allocated.
     */
    esp_err_t screen_start(const screen_config_t *config, screen_handle_t *out_handle);

    /**
     * Stop drawing and hand the panel over.
     *
     * For anything that wants the whole display to itself, a test pattern most
     * obviously. The face stays off the panel until screen_resume() puts it
     * back. Passing NULL does nothing, which keeps the call sites of a display
     * that never started simple.
     *
     * @param handle Screen handle, may be NULL.
     * @return ESP_OK on success.
     */
    esp_err_t screen_suspend(screen_handle_t handle);

    /**
     * Repaint the face and keep it current again.
     *
     * Repaints in full rather than by difference, so it is also the way back
     * from anything that drew over the display. Counts as activity, so a dimmed
     * panel comes back up to the brightness that was asked for.
     *
     * @param handle Screen handle, may be NULL.
     * @return ESP_OK on success.
     */
    esp_err_t screen_resume(screen_handle_t handle);

    /**
     * Report whether the face is currently suspended.
     *
     * @param handle Screen handle.
     * @param out_suspended Receives the state.
     * @return ESP_OK on success.
     */
    esp_err_t screen_is_suspended(screen_handle_t handle, bool *out_suspended);

    /**
     * Give the dog the whole panel for a while.
     *
     * Clears the face and puts the large portrait up, centred, for the given
     * time, then paints the face again. Called by the screen itself every time
     * the schedule moves on, which is just after a treat has been dispensed,
     * and available here for anything else that wants it. Counts as activity,
     * so a dimmed panel comes back up.
     *
     * Being over in seconds, this one carries no image sticking risk, which is
     * why it is allowed to be large and colourful when the badge on the face has
     * to be small and has to move.
     *
     * @param handle Screen handle, may be NULL.
     * @param duration_ms How long to hold it, SCREEN_CELEBRATE_HOLD to hold it
     *                    until told otherwise, SCREEN_CELEBRATE_UNTIL_QUIET to
     *                    hold it until the chime has finished, 0 to end one now.
     *                    Asking again while he is already up changes when he
     *                    comes off without flashing him.
     * @return ESP_OK on success.
     */
    esp_err_t screen_celebrate(screen_handle_t handle, uint32_t duration_ms);

    /**
     * Report where on the panel the face currently sits.
     *
     * Pixels from the middle, which is where the face would be if it never
     * moved. Diagnostic: it is how the console shows that the drift is running.
     *
     * @param handle Screen handle.
     * @param out_dx Receives the horizontal offset.
     * @param out_dy Receives the vertical offset.
     * @return ESP_OK on success.
     */
    esp_err_t screen_get_offset(screen_handle_t handle, int16_t *out_dx, int16_t *out_dy);

#ifdef __cplusplus
}
#endif
