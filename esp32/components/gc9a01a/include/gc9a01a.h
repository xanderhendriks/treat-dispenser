#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * ER-TFTM1.28-1: a 1.28 inch round IPS panel on a breakout board, driven by a
 * GC9A01A over 4-wire SPI. The controller always presents a square 240x240
 * frame memory; the glass only shows the 32.4 mm circle inscribed in it, so the
 * four corners of every buffer are written but never seen.
 *
 * The board wires the module's eight pin header as GND, VCC, SCL, SDA, RES, DC,
 * CS and BLK. There is no TE pin and no MISO on the header, so the driver never
 * waits for the panel's tearing signal and never reads a register back; the
 * frame is written blind and the only feedback that the link works is what
 * appears on the glass.
 */
#define GC9A01A_WIDTH  240
#define GC9A01A_HEIGHT 240

/** Pack 8-bit channels into the RGB565 word the panel expects. */
#define GC9A01A_RGB565(r, g, b) ((uint16_t) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

#define GC9A01A_BLACK   0x0000
#define GC9A01A_WHITE   0xFFFF
#define GC9A01A_RED     0xF800
#define GC9A01A_GREEN   0x07E0
#define GC9A01A_BLUE    0x001F
#define GC9A01A_CYAN    0x07FF
#define GC9A01A_MAGENTA 0xF81F
#define GC9A01A_YELLOW  0xFFE0

/*
 * Mid grey, and worth reaching for. The panel datasheet asks for medium grey
 * hues in the parts of a layout that stay put while the rest changes, and for
 * the colours either side of a long lived border to sit symmetrically around
 * mid grey. Both keep the DC component across the liquid crystal small, which
 * is what image sticking grows out of; see the screen care section below.
 */
#define GC9A01A_GREY_MID 0x8410

    typedef struct gc9a01a_t *gc9a01a_handle_t;

    typedef enum
    {
        GC9A01A_ROTATION_0 = 0,
        GC9A01A_ROTATION_90,
        GC9A01A_ROTATION_180,
        GC9A01A_ROTATION_270,
    } gc9a01a_rotation_t;

    /*
     * Source polarity pattern, from the GC9A01A frame rate register. The panel
     * runs a DC VCOM, so every pixel has its drive polarity flipped from frame
     * to frame to keep its average at zero; DINV chooses how that polarity is
     * also staggered across neighbouring dots within one frame. Finer
     * staggering leaves less of a standing field between adjacent pixels of
     * different colour, which is where sticking shows up first, and costs a
     * little more power in the source drivers.
     *
     * The vendor sample code for this module ships 4 dot inversion, so that is
     * what the driver uses unless asked otherwise.
     */
    typedef enum
    {
        GC9A01A_DOT_INVERSION_COLUMN = 0,
        GC9A01A_DOT_INVERSION_1_DOT,
        GC9A01A_DOT_INVERSION_2_DOT,
        GC9A01A_DOT_INVERSION_4_DOT,
        GC9A01A_DOT_INVERSION_8_DOT,
    } gc9a01a_dot_inversion_t;

    typedef struct
    {
        spi_host_device_t spi_host;

        int sck_gpio_num;
        int mosi_gpio_num;
        int cs_gpio_num;
        int dc_gpio_num;
        int rst_gpio_num;
        int backlight_gpio_num; /* -1 when BLK is not wired, which drops all backlight control */

        /*
         * 0 selects the 40 MHz the vendor sample code uses. The controller's
         * own write cycle bottoms out at 10 ns, so the silicon would take 80
         * MHz; the flex tail, the pin header and the 33R series resistors on
         * SCL and SDA are what actually set the ceiling here, so raising this
         * is a measurement rather than a guess.
         */
        int spi_clock_speed_hz;

        gc9a01a_rotation_t      rotation;
        gc9a01a_dot_inversion_t dot_inversion;

        /*
         * BLK is a logic input into the module's backlight switch, so dimming
         * is PWM from this side rather than the controller's own LEDPWM output,
         * which the eight pin header does not bring out. Pick a timer and
         * channel the motor driver is not already using. 0 selects 5 kHz, above
         * hearing and slow enough for a plain switch to keep up with.
         */
        ledc_timer_t   backlight_pwm_timer;
        ledc_channel_t backlight_pwm_channel;
        uint32_t       backlight_pwm_frequency_hz;

        /*
         * Size of the DMA staging buffer, in bytes. Pixels are byte swapped
         * into it a chunk at a time, so this trades internal RAM against the
         * number of SPI transactions per frame. 0 selects 4 kB, about eight
         * rows.
         */
        size_t transfer_chunk_bytes;
    } gc9a01a_config_t;

    /**
     * Initialize the GC9A01A panel driver.
     *
     * Creates the SPI bus on the given host, resets the panel, runs the vendor
     * initialization sequence for this module, applies the requested rotation
     * and clears the frame memory to black before turning the display on. The
     * backlight is left off, so nothing is shown until the caller has drawn its
     * first frame and called gc9a01a_set_backlight().
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success or an error from the SPI, GPIO or LEDC driver.
     */
    esp_err_t gc9a01a_init(const gc9a01a_config_t *config, gc9a01a_handle_t *out_handle);

    /**
     * Set the panel orientation.
     *
     * Takes effect immediately: what is already in frame memory is re-scanned
     * in the new orientation, so a repaint normally follows.
     *
     * @param handle Driver handle.
     * @param rotation Orientation to apply.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_set_rotation(gc9a01a_handle_t handle, gc9a01a_rotation_t rotation);

    /**
     * Read back the panel orientation.
     *
     * @param handle Driver handle.
     * @param out_rotation Receives the current orientation.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_get_rotation(gc9a01a_handle_t handle, gc9a01a_rotation_t *out_rotation);

    /**
     * Fill the whole frame memory with one colour.
     *
     * @param handle Driver handle.
     * @param color RGB565 colour, in host byte order.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_fill(gc9a01a_handle_t handle, uint16_t color);

    /**
     * Fill a rectangle with one colour.
     *
     * The rectangle is not clipped: anything reaching past the 240x240 frame
     * memory is rejected rather than trimmed, because on a round panel a
     * silently trimmed rectangle is far more likely to be a layout mistake than
     * an intent. Corners outside the visible circle are written and never seen.
     *
     * @param handle Driver handle.
     * @param x Left edge.
     * @param y Top edge.
     * @param width Width in pixels, at least 1.
     * @param height Height in pixels, at least 1.
     * @param color RGB565 colour, in host byte order.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when the rectangle does
     *         not fit.
     */
    esp_err_t gc9a01a_fill_rect(gc9a01a_handle_t handle, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                uint16_t color);

    /**
     * Copy an RGB565 image into a rectangle of the frame memory.
     *
     * Pixels are host byte order and row major, width entries per row. The
     * driver byte swaps them into its own DMA buffer on the way out, so the
     * source may live in flash and is never required to be DMA capable. Bounds
     * are checked the same way as gc9a01a_fill_rect().
     *
     * @param handle Driver handle.
     * @param x Left edge.
     * @param y Top edge.
     * @param width Width in pixels, at least 1.
     * @param height Height in pixels, at least 1.
     * @param pixels width * height RGB565 words.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when the rectangle does
     *         not fit.
     */
    esp_err_t gc9a01a_draw_bitmap(gc9a01a_handle_t handle, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                  const uint16_t *pixels);

    /**
     * Set the backlight duty cycle.
     *
     * The percentage is PWM duty rather than perceived brightness, which falls
     * off faster than the duty does at the bottom of the range. Also remembered
     * as the level to come back to after the screen care layer has dimmed or
     * blanked the panel.
     *
     * @param handle Driver handle.
     * @param percent 0-100.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG above 100,
     *         ESP_ERR_NOT_SUPPORTED when BLK is not wired.
     */
    esp_err_t gc9a01a_set_backlight(gc9a01a_handle_t handle, uint8_t percent);

    /**
     * Ramp the backlight to a duty cycle over the given time.
     *
     * Blocks until the ramp finishes, which is what makes it safe to paint
     * straight afterwards. A duration of 0 is the same as
     * gc9a01a_set_backlight().
     *
     * @param handle Driver handle.
     * @param percent 0-100.
     * @param duration_ms Ramp time in milliseconds.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG above 100,
     *         ESP_ERR_NOT_SUPPORTED when BLK is not wired.
     */
    esp_err_t gc9a01a_fade_backlight(gc9a01a_handle_t handle, uint8_t percent, uint32_t duration_ms);

    /**
     * Read back the backlight level the application last asked for.
     *
     * This is the requested level, not necessarily what is lit right now: the
     * screen care layer dims and blanks around it without changing it.
     *
     * @param handle Driver handle.
     * @param out_percent Receives 0-100.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_get_backlight(gc9a01a_handle_t handle, uint8_t *out_percent);

    /**
     * Turn the display output on or off.
     *
     * Off blanks the glass but leaves the panel powered and the frame memory
     * intact, so it comes back instantly. It does not drain the panel;
     * gc9a01a_sleep() does that.
     *
     * @param handle Driver handle.
     * @param on Whether to scan the panel.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_set_display_on(gc9a01a_handle_t handle, bool on);

    /**
     * Enter or leave sleep mode.
     *
     * Entering stops the DC/DC converter and the internal oscillator and, per
     * the controller datasheet, drains the charge off the panel, which is the
     * one thing this chip can do for a static image that has been sitting
     * there. The frame memory survives and the SPI interface stays alive, so
     * drawing while asleep is allowed and shows up on wake. Both directions
     * hold off for the datasheet's settling times, so a wake costs 120 ms.
     *
     * @param handle Driver handle.
     * @param sleep Whether to sleep.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_sleep(gc9a01a_handle_t handle, bool sleep);

    /**
     * Turn colour inversion on or off.
     *
     * The IPS panel on this module needs inversion on for colours to come out
     * right, which is how the driver leaves it. The control is here because
     * showing every pixel as its complement for a while is one way to unwind a
     * stuck image.
     *
     * @param handle Driver handle.
     * @param inverted Whether to invert.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_set_inversion(gc9a01a_handle_t handle, bool inverted);

    /**
     * Choose the source polarity pattern.
     *
     * @param handle Driver handle.
     * @param inversion Pattern to apply.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_set_dot_inversion(gc9a01a_handle_t handle, gc9a01a_dot_inversion_t inversion);

    /**
     * Shift the whole picture along the panel's scan direction.
     *
     * Moves where the controller starts reading frame memory out, so the image
     * slides without a repaint and without changing any drawing coordinate; the
     * rows pushed off one edge wrap round to the other. Used by the screen care
     * layer to keep a static border off the same row of pixels for hours on
     * end. On a rotated panel the shift still follows the panel's own scan
     * direction rather than the rotated one.
     *
     * @param handle Driver handle.
     * @param lines 0-239.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG beyond the panel height.
     */
    esp_err_t gc9a01a_set_scroll_offset(gc9a01a_handle_t handle, uint16_t lines);

    /*
     * ------------------------------------------------------------------------
     * Screen care
     * ------------------------------------------------------------------------
     *
     * The module datasheet is blunt about image sticking: a fixed picture
     * polarizes the liquid crystal, the crystal then cannot relax back, and a
     * ghost of the old picture stays visible under the new one. It is inherent
     * to the technology, it is explicitly not covered by warranty, and the
     * vendor's advice is entirely about how the panel is driven:
     *
     *   - do not leave a fixed image up for more than two hours, and as little
     *     as thirty minutes when the panel is running warm,
     *   - dim or blank after five to ten idle minutes, to black or mid grey,
     *   - power the panel down over long idle stretches,
     *   - nudge long lived borders sideways now and then,
     *   - prefer mid grey and block fills over hard lines for anything static.
     *
     * A dispenser on a shelf showing the same next feeding time all day is
     * exactly the case they are describing, so the policy lives here rather
     * than being left to whoever draws the screen. An optional task watches how
     * long the picture has been unchanged and works down that list: it nudges
     * the picture, dims it, then paints black and sleeps the panel, which also
     * drains it. Whoever draws calls gc9a01a_care_touch() first; that resets
     * the clock and brings the panel back up, so the drawing code never has to
     * know any of this happened.
     *
     * The recovery half of the advice is gc9a01a_care_soak(): black on a
     * powered panel for hours, which is how a ghost that has already formed
     * gets unwound.
     */

    /** Timeout value that switches a screen care step off entirely. */
#define GC9A01A_CARE_NEVER UINT32_MAX

    typedef enum
    {
        GC9A01A_CARE_STATE_ACTIVE = 0, /* showing what was drawn, at the requested backlight */
        GC9A01A_CARE_STATE_NUDGED,     /* still lit, picture walked off its original rows */
        GC9A01A_CARE_STATE_DIMMED,     /* backlight down, picture still up */
        GC9A01A_CARE_STATE_BLANKED,    /* black, backlight off, panel asleep and draining */
        GC9A01A_CARE_STATE_SOAKING,    /* deliberate black soak to unwind a ghost */
    } gc9a01a_care_state_t;

    typedef struct
    {
        /*
         * How long the picture has to sit unchanged before each step. 0 selects
         * the default and GC9A01A_CARE_NEVER switches the step off. The
         * defaults follow the datasheet: a nudge every minute, dim at five
         * minutes, black and asleep at ten.
         */
        uint32_t nudge_after_ms;
        uint32_t dim_after_ms;
        uint32_t blank_after_ms;

        /*
         * How long a static picture may stand before it is worth complaining
         * about in the log. 0 selects the two hours the datasheet names, which
         * is only ever reached with dimming and blanking both switched off.
         */
        uint32_t warn_after_ms;

        uint8_t dim_backlight_pct;  /* 0 selects 20 percent */
        uint8_t nudge_amplitude_px; /* 0 selects 2 pixels; the walk runs 0..amplitude and back */

        uint32_t task_stack_size;
        uint32_t task_priority;
    } gc9a01a_care_config_t;

    typedef struct
    {
        gc9a01a_care_state_t state;
        uint32_t             static_ms;       /* since the last gc9a01a_care_touch() */
        uint16_t             nudge_offset_px; /* scroll offset currently applied */
        uint32_t             soak_remaining_s;
    } gc9a01a_care_status_t;

    /**
     * Start the screen care task.
     *
     * @param handle Driver handle.
     * @param config Policy, or NULL for the datasheet's own advice throughout.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when already started.
     */
    esp_err_t gc9a01a_care_start(gc9a01a_handle_t handle, const gc9a01a_care_config_t *config);

    /**
     * Tell the screen care layer that the picture is about to change.
     *
     * Resets the static clock, undoes any nudge so that drawing coordinates
     * land where they are expected, and wakes and relights a panel that had
     * been blanked. Blocks for the panel's 120 ms wake when it comes to that
     * and returns immediately otherwise, so it is cheap enough to call before
     * every update. A soak in progress is left alone: the picture goes into
     * frame memory and appears when the soak ends.
     *
     * Safe to call before gc9a01a_care_start(), where it does nothing.
     *
     * @param handle Driver handle.
     * @return ESP_OK on success.
     */
    esp_err_t gc9a01a_care_touch(gc9a01a_handle_t handle);

    /**
     * Soak the panel in black to unwind an image that has already stuck.
     *
     * Paints black, kills the backlight and holds the panel awake and scanning
     * that black for the requested time, which is the datasheet's own recipe:
     * all black for four to six hours, and quicker if the panel is warm.
     * Deliberately not sleep mode, so that the crystal is actively driven to
     * its relaxed state rather than just left unpowered. A duration of 0 stops
     * a soak that is running.
     *
     * @param handle Driver handle.
     * @param minutes How long to soak.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when screen care is not
     *         running.
     */
    esp_err_t gc9a01a_care_soak(gc9a01a_handle_t handle, uint32_t minutes);

    /**
     * Read what the screen care layer is doing.
     *
     * @param handle Driver handle.
     * @param out_status Receives the current state.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when screen care is not
     *         running.
     */
    esp_err_t gc9a01a_care_get_status(gc9a01a_handle_t handle, gc9a01a_care_status_t *out_status);

#ifdef __cplusplus
}
#endif
