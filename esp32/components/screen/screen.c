#include "screen.h"

#include <inttypes.h>
#include <stdlib.h>
#include <time.h>

#include "bt_icon.h"
#include "dog_badge.h"
#include "dog_large.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pixelfont.h"
#include "sevenseg.h"

#define SCREEN_DEFAULT_STACK_SIZE 3072

/*
 * Below the scheduler and the drum, and below the console too. Nothing on this
 * board should ever wait on a clock face.
 */
#define SCREEN_DEFAULT_PRIORITY 2

/*
 * Four times a second. The seconds digit only moves once a second, but sampling
 * faster than that keeps the turnover from wandering: the tick period drifts
 * against the RTC by however long the work takes, and at this rate the drift
 * never adds up to a visibly skipped beat. A tick costs two short I2C reads and,
 * in the usual second, seven small rectangles for the one digit that changed.
 */
#define SCREEN_TICK_MS 250

#define SCREEN_CLOCK_DIGITS 4 /* h h m m, the leading hour digit blank before ten */
#define SCREEN_COUNT_DIGITS 6 /* HH MM SS */

#define SCREEN_SECONDS_PER_DAY    86400
#define SCREEN_HOURS_PER_MERIDIEM 12

/* Space between glyphs in a row, between the time and its AM or PM, and between the three rows */
#define SCREEN_CLOCK_GAP  2
#define SCREEN_COUNT_GAP  3
#define SCREEN_AMPM_GAP   5
#define SCREEN_AMPM_SCALE 2
#define SCREEN_ICON_GAP   20 /* from the dog, which puts the icon about halfway out to the bezel */
#define SCREEN_GAP_ABOVE  18
#define SCREEN_GAP_BELOW  18

/*
 * Blink rates for the Bluetooth badge, as half periods. The pairing window keeps
 * the brisk pulse: it is the state that wants noticing and the only one with a
 * deadline. A phone actually connected is a calmer fact and gets half the rate,
 * so the two are told apart by which one is hurrying.
 *
 * Neither can go much quicker than the brisk one. The face is painted four
 * times a second, so a half period shorter than two ticks would be sampled at
 * about its own rate and beat against it, and the blink would stutter rather
 * than keep time. Five hundred is that floor.
 */
#define SCREEN_BT_PAIRING_BLINK_MS   500
#define SCREEN_BT_CONNECTED_BLINK_MS 1000

/*
 * The dog gets the whole panel whenever the drum turns, from the moment it
 * starts until the chime it ends on has finished, so how long that is comes from
 * the move and the melody rather than from a number here. Being over in seconds
 * either way, it carries no image sticking risk at all, which is why this one is
 * allowed to be large and colourful when the badge on the face has to be smaller
 * and has to move.
 */

/*
 * Black background, because a normally black panel holds its liquid crystal
 * relaxed there and the datasheet names black as a fine choice for a screen
 * that idles. The countdown is white for legibility at a distance and the clock
 * a softer grey, which puts the two in their proper order at a glance and keeps
 * the smaller digits from shouting.
 */
#define SCREEN_BACKGROUND  GC9A01A_BLACK
#define SCREEN_COUNT_COLOR GC9A01A_WHITE
#define SCREEN_CLOCK_COLOR GC9A01A_RGB565(150, 150, 150)

/*
 * The countdown is sized to leave room to move rather than to fill the glass.
 * At 28 by 48 it came within two pixels of the bezel, which is less than one
 * stroke and so no use at all against image sticking: a shift that small leaves
 * most of every segment standing on the pixels it was already on.
 */
static const sevenseg_style_t s_clock_style = {.width = 14, .height = 23, .stroke = 3};
static const sevenseg_style_t s_count_style = {.width = 24, .height = 41, .stroke = 5};

/*
 * Where the face sits is walked round an eight point ring, one step at a time.
 * A ring rather than a crawl because what matters is that a lit pixel stops
 * being lit, and only a step wider than the stroke does that; and eight points
 * rather than two so that no position is occupied more than an eighth of the
 * time.
 *
 * The table is one cycle of a sine scaled to 64. Reading it a quarter cycle
 * further along for the second axis gives the cosine, and the two together an
 * octagon.
 */
#define SCREEN_DRIFT_STEPS      8
#define SCREEN_DRIFT_SINE_SCALE 64

static const int8_t s_drift_sine[SCREEN_DRIFT_STEPS] = {0, 45, 64, 45, 0, -45, -64, -45};

#define SCREEN_DRIFT_DEFAULT_INTERVAL_MS 60000

/*
 * Kept back from the largest radius the glass would allow, because the screen
 * care layer slides the whole readout by up to two rows of its own on top of
 * this, and because a corner is measured from its own pixel rather than its
 * outer edge.
 */
#define SCREEN_DRIFT_MARGIN_PX 3

/* Dog, countdown, clock, and the Bluetooth icon beside the dog */
#define SCREEN_ROW_COUNT 4

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} screen_rect_t;

/*
 * The icon is one mark in one colour, so all a state has to say is whether it
 * is on the glass. Steady on means the dispenser talks Bluetooth, a blink means
 * either a phone is connected or a new one may pair, and gone means no radio
 * came up, which is worth showing by showing nothing: an icon that is there
 * whether or not the thing works says nothing at all.
 */
typedef enum
{
    SCREEN_BT_HIDDEN = 0, /* no radio, or the dark half of the pairing blink */
    SCREEN_BT_SHOWN,
} screen_bt_t;

typedef struct screen_t
{
    gc9a01a_handle_t    display;
    rv3028_handle_t     rtc;
    scheduler_handle_t  schedule;
    ble_remote_handle_t ble;
    max98357a_handle_t  audio;

    TaskHandle_t      task;
    SemaphoreHandle_t lock;

    bool suspended;
    bool repaint; /* clear the whole panel and draw everything */
    bool moved;   /* the face changed position: wipe where it was, then draw everything */

    /* What is on the glass, so that a tick only has to touch what moved */
    uint8_t     clock_shown[SCREEN_CLOCK_DIGITS];
    uint8_t     count_shown[SCREEN_COUNT_DIGITS];
    char        ampm_shown; /* 'A', 'P', or 0 for nothing */
    screen_bt_t bt_shown;

    scheduler_slot_t slot_shown;
    bool             slot_known;

    /* Laid out once from the styles and the asset sizes, rather than kept as a table of offsets that could drift */
    uint16_t badge_x;
    uint16_t badge_y;
    uint16_t count_y;
    uint16_t clock_y;
    uint16_t count_x[SCREEN_COUNT_DIGITS];
    uint16_t clock_x[SCREEN_CLOCK_DIGITS];
    uint16_t count_colon_x[2];
    uint16_t clock_colon_x;
    uint16_t ampm_x;
    uint16_t ampm_y;
    uint16_t ampm_width;
    uint16_t ampm_height;
    uint16_t icon_x;
    uint16_t icon_y;

    /*
     * Everything drawn, which is what the drift is measured against, and its
     * bounding box, which is what a move wipes.
     */
    screen_rect_t rows[SCREEN_ROW_COUNT];
    screen_rect_t plate;

    uint32_t drift_interval_ms;
    uint8_t  drift_radius_px;
    uint8_t  drift_step;
    int16_t  drift_dx;
    int16_t  drift_dy;
    int16_t  wiped_dx; /* where the face was before the last step, so it can be cleaned up */
    int16_t  wiped_dy;
    int64_t  drifted_us;

    bool     celebrate_pending; /* picked up on the next tick, so this can be set with the lock held */
    uint32_t celebrate_ms;
    int64_t  celebrate_until_us;
    bool     celebrate_until_quiet; /* also come off once the chime has finished */
} screen_ctx_t;

static const char *TAG = "screen";

static void        screen_task(void *arg);
static void        screen_tick(screen_ctx_t *ctx);
static void        screen_layout(screen_ctx_t *ctx);
static void        screen_paint(screen_ctx_t *ctx, const uint8_t *clock_digits, char ampm, const uint8_t *count_digits,
                                screen_bt_t bluetooth);
static void        screen_paint_celebration(screen_ctx_t *ctx);
static bool        screen_read_clock(screen_ctx_t *ctx, uint8_t *out_digits, char *out_ampm, struct tm *out_time);
static bool        screen_read_countdown(screen_ctx_t *ctx, const struct tm *now, uint8_t *out_digits);
static uint32_t    screen_seconds_until(const struct tm *now, const scheduler_slot_t *slot);
static void        screen_fill_digits(uint8_t *digits, size_t count, uint8_t value);
static bool        screen_panel_is_dark(screen_ctx_t *ctx);
static screen_bt_t screen_read_bluetooth(screen_ctx_t *ctx, int64_t now_us);
static bool        screen_blink_on(int64_t now_us, uint32_t half_period_ms);
static bool        screen_audio_busy(screen_ctx_t *ctx);
static void        screen_advance_drift(screen_ctx_t *ctx);
static int16_t     screen_drift_offset(uint8_t radius, uint8_t step, uint8_t phase);
static bool        screen_drift_fits(const screen_rect_t *rows, size_t count, uint8_t radius);
static uint8_t     screen_max_drift_radius(const screen_rect_t *rows, size_t count);

esp_err_t screen_start(const screen_config_t *config, screen_handle_t *out_handle)
{
    screen_ctx_t *ctx;
    uint8_t       radius;
    uint8_t       allowed;

    if (!config || !config->display_handle || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(screen_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->display  = config->display_handle;
    ctx->rtc      = config->rtc_handle;
    ctx->schedule = config->schedule_handle;
    ctx->ble      = config->ble_handle;
    ctx->audio    = config->audio_handle;
    ctx->repaint  = true;

    ctx->drift_interval_ms = config->drift_interval_ms ? config->drift_interval_ms : SCREEN_DRIFT_DEFAULT_INTERVAL_MS;

    ctx->lock = xSemaphoreCreateMutex();
    if (!ctx->lock)
    {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    screen_layout(ctx);

    /*
     * By default the layout decides: whatever slack is left between the rows
     * and the glass is what the face gets to move in, so growing the badge or a
     * digit costs travel and nothing has to be re-tuned by hand. An explicit
     * radius is still clamped to that, rather than rejected, so it can never
     * quietly push the face under the bezel.
     */
    allowed = screen_max_drift_radius(ctx->rows, SCREEN_ROW_COUNT);
    radius  = config->drift_radius_px ? config->drift_radius_px : allowed;
    if (radius > allowed)
    {
        ESP_LOGW(TAG, "Face can only move %u px inside the glass, not the %u asked for", (unsigned) allowed,
                 (unsigned) radius);
        radius = allowed;
    }
    ctx->drift_radius_px = radius;
    ctx->drifted_us      = esp_timer_get_time();

    if (config->splash)
    {
        ctx->celebrate_pending = true;
        ctx->celebrate_ms      = SCREEN_CELEBRATE_HOLD;
    }

    /* First frame before the task exists, so whoever brings the backlight up finds something already there */
    screen_tick(ctx);

    if (xTaskCreate(screen_task, "screen",
                    config->task_stack_size ? config->task_stack_size : SCREEN_DEFAULT_STACK_SIZE, ctx,
                    config->task_priority ? (UBaseType_t) config->task_priority : SCREEN_DEFAULT_PRIORITY,
                    &ctx->task) != pdPASS)
    {
        vSemaphoreDelete(ctx->lock);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    if (!ctx->rtc)
    {
        ESP_LOGW(TAG, "No RTC, the clock will show dashes");
    }
    if (!ctx->schedule)
    {
        ESP_LOGW(TAG, "No dispensing schedule, the countdown will show dashes");
    }

    if (ctx->drift_interval_ms == SCREEN_DRIFT_NEVER || ctx->drift_radius_px == 0)
    {
        ESP_LOGW(TAG, "The face will stay put, which leaves image sticking to the screen care layer alone");
    }
    else
    {
        ESP_LOGI(TAG, "Showing the dog, the countdown and the clock, moving %u px every %" PRIu32 " s",
                 (unsigned) ctx->drift_radius_px, ctx->drift_interval_ms / 1000);
    }

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t screen_suspend(screen_handle_t handle)
{
    if (!handle)
    {
        return ESP_OK;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->suspended = true;
    xSemaphoreGive(handle->lock);

    return ESP_OK;
}

esp_err_t screen_resume(screen_handle_t handle)
{
    if (!handle)
    {
        return ESP_OK;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->suspended = false;
    handle->repaint   = true;

    /*
     * Ends a celebration, and the boot splash with it. Without this, asking for
     * the face back while the dog is up would set the repaint and then be
     * stepped over every tick for as long as he is held, which for the splash is
     * until the drum has finished; anything drawn over the panel in the meantime
     * would simply stay there.
     */
    handle->celebrate_pending     = false;
    handle->celebrate_until_us    = 0;
    handle->celebrate_until_quiet = false;

    xSemaphoreGive(handle->lock);

    /* Somebody asked for the face back, which is an event worth relighting a dimmed panel for */
    gc9a01a_care_touch(handle->display);

    screen_tick(handle);

    return ESP_OK;
}

esp_err_t screen_is_suspended(screen_handle_t handle, bool *out_suspended)
{
    if (!handle || !out_suspended)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    *out_suspended = handle->suspended;
    xSemaphoreGive(handle->lock);

    return ESP_OK;
}

esp_err_t screen_celebrate(screen_handle_t handle, uint32_t duration_ms)
{
    if (!handle)
    {
        return ESP_OK;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    if (duration_ms == 0)
    {
        /* Cut it short, and put the face back */
        handle->celebrate_pending     = false;
        handle->celebrate_until_us    = 0;
        handle->celebrate_until_quiet = false;
        handle->repaint               = true;
    }
    else
    {
        handle->celebrate_pending = true;
        handle->celebrate_ms      = duration_ms;
    }

    xSemaphoreGive(handle->lock);

    /* Worth relighting a dimmed panel for, whoever asked and whatever the reason */
    gc9a01a_care_touch(handle->display);

    return ESP_OK;
}

esp_err_t screen_get_offset(screen_handle_t handle, int16_t *out_dx, int16_t *out_dy)
{
    if (!handle || !out_dx || !out_dy)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    *out_dx = handle->drift_dx;
    *out_dy = handle->drift_dy;
    xSemaphoreGive(handle->lock);

    return ESP_OK;
}

static void screen_task(void *arg)
{
    screen_ctx_t *ctx = arg;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(SCREEN_TICK_MS));
        screen_tick(ctx);
    }
}

static void screen_tick(screen_ctx_t *ctx)
{
    uint8_t     clock_digits[SCREEN_CLOCK_DIGITS];
    uint8_t     count_digits[SCREEN_COUNT_DIGITS];
    char        ampm = 0;
    screen_bt_t bluetooth;
    struct tm   now;
    int64_t     now_us;
    bool        time_known;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);

    if (ctx->suspended)
    {
        xSemaphoreGive(ctx->lock);
        return;
    }

    /*
     * Nothing is drawn while the panel is asleep or deliberately soaking in
     * black. Painting over a soak would defeat the whole point of it, and there
     * is no sense writing frames into a panel that is not scanning them; the
     * face is marked for a full repaint so that it comes back whole.
     */
    if (screen_panel_is_dark(ctx))
    {
        ctx->repaint = true;
        xSemaphoreGive(ctx->lock);
        return;
    }

    now_us = esp_timer_get_time();

    /*
     * A fresh request is taken first, so that one arriving while the dog is
     * already up only moves when he comes off. Repainting in that case would
     * clear the panel and draw the same frame again, which reads as a flinch;
     * the move observer does exactly this, asking to hold as the drum starts and
     * then to wait for the chime once it has stopped.
     */
    if (ctx->celebrate_pending)
    {
        bool showing = ctx->celebrate_until_us != 0;

        ctx->celebrate_pending     = false;
        ctx->celebrate_until_quiet = ctx->celebrate_ms == SCREEN_CELEBRATE_UNTIL_QUIET;
        ctx->celebrate_until_us    = (ctx->celebrate_ms == SCREEN_CELEBRATE_HOLD || ctx->celebrate_until_quiet)
                                         ? INT64_MAX
                                         : now_us + (int64_t) ctx->celebrate_ms * 1000;

        if (!showing)
        {
            screen_paint_celebration(ctx);
        }

        xSemaphoreGive(ctx->lock);
        return;
    }

    /*
     * While the dog is up the face is not drawn at all; his frame is already on
     * the panel and there is nothing to keep current behind it. When his time
     * is up the face comes back whole, because he covered most of it.
     */
    if (ctx->celebrate_until_us != 0)
    {
        bool done = now_us >= ctx->celebrate_until_us;

        if (!done && ctx->celebrate_until_quiet)
        {
            done = !screen_audio_busy(ctx);
        }

        if (!done)
        {
            xSemaphoreGive(ctx->lock);
            return;
        }

        ctx->celebrate_until_us    = 0;
        ctx->celebrate_until_quiet = false;
        ctx->repaint               = true;
    }

    if (ctx->drift_interval_ms != SCREEN_DRIFT_NEVER && ctx->drift_radius_px > 0 &&
        (uint32_t) ((now_us - ctx->drifted_us) / 1000) >= ctx->drift_interval_ms)
    {
        screen_advance_drift(ctx);
    }

    time_known = screen_read_clock(ctx, clock_digits, &ampm, &now);

    if (!time_known || !screen_read_countdown(ctx, &now, count_digits))
    {
        screen_fill_digits(count_digits, SCREEN_COUNT_DIGITS, SEVENSEG_DASH);
    }

    bluetooth = screen_read_bluetooth(ctx, now_us);

    screen_paint(ctx, clock_digits, ampm, count_digits, bluetooth);

    xSemaphoreGive(ctx->lock);
}

/*
 * Three rows, top to bottom: the dog, the countdown, then the time with its AM
 * or PM.
 *
 * The stack is centred as a whole rather than the countdown being pinned to the
 * middle of the panel. Pinning it looks wrong: the dog is nearly four times the
 * height of the clock row, so holding the countdown at the centre leaves the top
 * of the dog crowding the bezel and a large empty arc under the time, and the
 * face reads as having slid upwards. Centring the stack gives the same air above
 * and below. The countdown is still the middle row and still the thing the eye
 * goes to, which is what being in the middle was for.
 *
 * The sizes and the gaps are what the travel will pay for and no more. With the
 * dog at 67 by 88 and eighteen pixels between rows the layout allows exactly
 * twelve pixels of movement, which is the figure the drift wants; a larger dog
 * or wider gaps start eating into that, and screen_max_drift_radius() will say
 * so rather than let anything slide under the bezel.
 *
 * The rows are kept individually rather than only as a bounding box, and that
 * matters more than it looks. The countdown is 175 wide and the other two about
 * half that, so a bounding box charges the dog and the clock for corners that
 * hold nothing — at the top and bottom of a circle, which is exactly where the
 * room runs out. Measuring the rows themselves is what makes a bigger dog and
 * more travel affordable at the same time.
 */
static void screen_layout(screen_ctx_t *ctx)
{
    uint16_t count_width;
    uint16_t time_width;
    uint16_t clock_width;
    uint16_t content_height;
    uint16_t x;
    int32_t  left   = GC9A01A_WIDTH;
    int32_t  top    = GC9A01A_HEIGHT;
    int32_t  right  = 0;
    int32_t  bottom = 0;

    count_width = SCREEN_COUNT_DIGITS * s_count_style.width + 2 * sevenseg_colon_width(&s_count_style) +
                  (SCREEN_COUNT_DIGITS + 1) * SCREEN_COUNT_GAP;
    time_width = SCREEN_CLOCK_DIGITS * s_clock_style.width + sevenseg_colon_width(&s_clock_style) +
                 SCREEN_CLOCK_DIGITS * SCREEN_CLOCK_GAP;

    ctx->ampm_width  = pixelfont_width(SCREEN_AMPM_SCALE, "AM");
    ctx->ampm_height = pixelfont_height(SCREEN_AMPM_SCALE);
    clock_width      = time_width + SCREEN_AMPM_GAP + ctx->ampm_width;

    content_height =
        DOG_BADGE_HEIGHT + SCREEN_GAP_ABOVE + s_count_style.height + SCREEN_GAP_BELOW + s_clock_style.height;

    ctx->badge_y = (GC9A01A_HEIGHT - content_height) / 2;
    ctx->count_y = ctx->badge_y + DOG_BADGE_HEIGHT + SCREEN_GAP_ABOVE;
    ctx->clock_y = ctx->count_y + s_count_style.height + SCREEN_GAP_BELOW;
    ctx->badge_x = (GC9A01A_WIDTH - DOG_BADGE_WIDTH) / 2;

    /*
     * The Bluetooth icon goes in the empty ground to the left of the dog, level
     * with the middle of his head. It is a status mark rather than part of the
     * composition, so the dog stays centred and the icon takes space that was
     * doing nothing; it also sits where the circle is at its widest, which is
     * why it costs no travel at all.
     */
    ctx->icon_x = (uint16_t) (ctx->badge_x - SCREEN_ICON_GAP - BT_ICON_WIDTH);
    ctx->icon_y = (uint16_t) (ctx->badge_y + (DOG_BADGE_HEIGHT - BT_ICON_HEIGHT) / 2);

    /* HH:MM:SS */
    x               = (GC9A01A_WIDTH - count_width) / 2;
    ctx->count_x[0] = x;
    x += s_count_style.width + SCREEN_COUNT_GAP;
    ctx->count_x[1] = x;
    x += s_count_style.width + SCREEN_COUNT_GAP;
    ctx->count_colon_x[0] = x;
    x += sevenseg_colon_width(&s_count_style) + SCREEN_COUNT_GAP;
    ctx->count_x[2] = x;
    x += s_count_style.width + SCREEN_COUNT_GAP;
    ctx->count_x[3] = x;
    x += s_count_style.width + SCREEN_COUNT_GAP;
    ctx->count_colon_x[1] = x;
    x += sevenseg_colon_width(&s_count_style) + SCREEN_COUNT_GAP;
    ctx->count_x[4] = x;
    x += s_count_style.width + SCREEN_COUNT_GAP;
    ctx->count_x[5] = x;

    /* h:mm then AM or PM, the label centred against the digits it follows */
    x               = (GC9A01A_WIDTH - clock_width) / 2;
    ctx->clock_x[0] = x;
    x += s_clock_style.width + SCREEN_CLOCK_GAP;
    ctx->clock_x[1] = x;
    x += s_clock_style.width + SCREEN_CLOCK_GAP;
    ctx->clock_colon_x = x;
    x += sevenseg_colon_width(&s_clock_style) + SCREEN_CLOCK_GAP;
    ctx->clock_x[2] = x;
    x += s_clock_style.width + SCREEN_CLOCK_GAP;
    ctx->clock_x[3] = x;
    x += s_clock_style.width;
    ctx->ampm_x = x + SCREEN_AMPM_GAP;

    /*
     * Sat on the same bottom edge as the digits rather than centred against
     * them. Centred, a label two thirds their height floats above the line they
     * stand on and reads as having come adrift; a shared baseline is what makes
     * the two look like one row.
     */
    ctx->ampm_y = ctx->clock_y + s_clock_style.height - ctx->ampm_height;

    ctx->rows[0] = (screen_rect_t){ctx->badge_x, ctx->badge_y, DOG_BADGE_WIDTH, DOG_BADGE_HEIGHT};
    ctx->rows[1] = (screen_rect_t){(uint16_t) ((GC9A01A_WIDTH - count_width) / 2), ctx->count_y, count_width,
                                   s_count_style.height};
    ctx->rows[2] = (screen_rect_t){(uint16_t) ((GC9A01A_WIDTH - clock_width) / 2), ctx->clock_y, clock_width,
                                   s_clock_style.height};
    ctx->rows[3] = (screen_rect_t){ctx->icon_x, ctx->icon_y, BT_ICON_WIDTH, BT_ICON_HEIGHT};

    for (size_t i = 0; i < SCREEN_ROW_COUNT; i++)
    {
        left   = ctx->rows[i].x < left ? ctx->rows[i].x : left;
        top    = ctx->rows[i].y < top ? ctx->rows[i].y : top;
        right  = ctx->rows[i].x + ctx->rows[i].width > right ? ctx->rows[i].x + ctx->rows[i].width : right;
        bottom = ctx->rows[i].y + ctx->rows[i].height > bottom ? ctx->rows[i].y + ctx->rows[i].height : bottom;
    }

    ctx->plate = (screen_rect_t){(uint16_t) left, (uint16_t) top, (uint16_t) (right - left), (uint16_t) (bottom - top)};
}

/*
 * A digit is only touched when its value has moved, which in the usual second
 * is one digit and seven small rectangles. The dog, the colons and the
 * background go on only when the whole face is being painted, because nothing
 * else ever writes over them; AM and PM change twice a day.
 *
 * A step of the drift wipes the plate it used to occupy rather than the whole
 * panel. Wiping all 240 by 240 would be about twice the pixels and would show
 * as a flicker once a minute; this way the gap between the old face going and
 * the new one arriving is a few milliseconds, inside a frame.
 */
static void screen_paint(screen_ctx_t *ctx, const uint8_t *clock_digits, char ampm, const uint8_t *count_digits,
                         screen_bt_t bluetooth)
{
    bool full = ctx->repaint || ctx->moved;

    if (ctx->repaint)
    {
        gc9a01a_fill(ctx->display, SCREEN_BACKGROUND);
    }
    else if (ctx->moved)
    {
        gc9a01a_fill_rect(ctx->display, (uint16_t) (ctx->plate.x + ctx->wiped_dx),
                          (uint16_t) (ctx->plate.y + ctx->wiped_dy), ctx->plate.width, ctx->plate.height,
                          SCREEN_BACKGROUND);
    }

    if (full)
    {
        gc9a01a_draw_bitmap(ctx->display, (uint16_t) (ctx->badge_x + ctx->drift_dx),
                            (uint16_t) (ctx->badge_y + ctx->drift_dy), DOG_BADGE_WIDTH, DOG_BADGE_HEIGHT, dog_badge);

        sevenseg_draw_colon(ctx->display, (uint16_t) (ctx->count_colon_x[0] + ctx->drift_dx),
                            (uint16_t) (ctx->count_y + ctx->drift_dy), &s_count_style, SCREEN_COUNT_COLOR);
        sevenseg_draw_colon(ctx->display, (uint16_t) (ctx->count_colon_x[1] + ctx->drift_dx),
                            (uint16_t) (ctx->count_y + ctx->drift_dy), &s_count_style, SCREEN_COUNT_COLOR);
        sevenseg_draw_colon(ctx->display, (uint16_t) (ctx->clock_colon_x + ctx->drift_dx),
                            (uint16_t) (ctx->clock_y + ctx->drift_dy), &s_clock_style, SCREEN_CLOCK_COLOR);
    }

    for (size_t i = 0; i < SCREEN_COUNT_DIGITS; i++)
    {
        if (!full && ctx->count_shown[i] == count_digits[i])
        {
            continue;
        }

        sevenseg_draw_digit(ctx->display, (uint16_t) (ctx->count_x[i] + ctx->drift_dx),
                            (uint16_t) (ctx->count_y + ctx->drift_dy), &s_count_style, count_digits[i],
                            SCREEN_COUNT_COLOR, SCREEN_BACKGROUND);
        ctx->count_shown[i] = count_digits[i];
    }

    for (size_t i = 0; i < SCREEN_CLOCK_DIGITS; i++)
    {
        if (!full && ctx->clock_shown[i] == clock_digits[i])
        {
            continue;
        }

        sevenseg_draw_digit(ctx->display, (uint16_t) (ctx->clock_x[i] + ctx->drift_dx),
                            (uint16_t) (ctx->clock_y + ctx->drift_dy), &s_clock_style, clock_digits[i],
                            SCREEN_CLOCK_COLOR, SCREEN_BACKGROUND);
        ctx->clock_shown[i] = clock_digits[i];
    }

    if (full || ctx->bt_shown != bluetooth)
    {
        uint16_t x = (uint16_t) (ctx->icon_x + ctx->drift_dx);
        uint16_t y = (uint16_t) (ctx->icon_y + ctx->drift_dy);

        if (bluetooth == SCREEN_BT_SHOWN)
        {
            gc9a01a_draw_bitmap(ctx->display, x, y, BT_ICON_WIDTH, BT_ICON_HEIGHT, bt_icon);
        }
        else
        {
            /* No radio, or the dark half of the blink: the ground goes back to being background */
            gc9a01a_fill_rect(ctx->display, x, y, BT_ICON_WIDTH, BT_ICON_HEIGHT, SCREEN_BACKGROUND);
        }

        ctx->bt_shown = bluetooth;
    }

    if (full || ctx->ampm_shown != ampm)
    {
        uint16_t x = (uint16_t) (ctx->ampm_x + ctx->drift_dx);
        uint16_t y = (uint16_t) (ctx->ampm_y + ctx->drift_dy);

        if (ampm)
        {
            pixelfont_draw(ctx->display, x, y, SCREEN_AMPM_SCALE, ampm == 'P' ? "PM" : "AM", SCREEN_CLOCK_COLOR,
                           SCREEN_BACKGROUND);
        }
        else
        {
            /* No time to label, so the space goes back to being background */
            gc9a01a_fill_rect(ctx->display, x, y, ctx->ampm_width, ctx->ampm_height, SCREEN_BACKGROUND);
        }

        ctx->ampm_shown = ampm;
    }

    ctx->repaint = false;
    ctx->moved   = false;
}

/*
 * The dog, centred, with the rest of the panel cleared around him. He does not
 * take part in the drift: he is on screen for seconds rather than hours, so
 * there is nothing for a shift to protect, and dead centre is where he belongs.
 */
static void screen_paint_celebration(screen_ctx_t *ctx)
{
    gc9a01a_fill(ctx->display, SCREEN_BACKGROUND);
    gc9a01a_draw_bitmap(ctx->display, (GC9A01A_WIDTH - DOG_LARGE_WIDTH) / 2, (GC9A01A_HEIGHT - DOG_LARGE_HEIGHT) / 2,
                        DOG_LARGE_WIDTH, DOG_LARGE_HEIGHT, dog_large);
}

/*
 * Twelve hour time, this being a dispenser for an Australian dog. Midnight and
 * noon both read as twelve, and the leading hour digit is blanked rather than
 * zeroed before ten, which is what a clock does and what SEVENSEG_BLANK is for.
 * Dashes and no label stand in when there is no RTC or it never had its time
 * set.
 */
static bool screen_read_clock(screen_ctx_t *ctx, uint8_t *out_digits, char *out_ampm, struct tm *out_time)
{
    bool valid = false;
    int  hour;

    *out_ampm = 0;

    if (!ctx->rtc || rv3028_is_time_valid(ctx->rtc, &valid) != ESP_OK || !valid ||
        rv3028_get_time(ctx->rtc, out_time) != ESP_OK)
    {
        screen_fill_digits(out_digits, SCREEN_CLOCK_DIGITS, SEVENSEG_DASH);
        return false;
    }

    hour = out_time->tm_hour % SCREEN_HOURS_PER_MERIDIEM;
    if (hour == 0)
    {
        hour = SCREEN_HOURS_PER_MERIDIEM;
    }

    out_digits[0] = hour >= 10 ? 1 : SEVENSEG_BLANK;
    out_digits[1] = (uint8_t) (hour % 10);
    out_digits[2] = (uint8_t) (out_time->tm_min / 10);
    out_digits[3] = (uint8_t) (out_time->tm_min % 10);
    *out_ampm     = out_time->tm_hour < SCREEN_HOURS_PER_MERIDIEM ? 'A' : 'P';

    return true;
}

static bool screen_read_countdown(screen_ctx_t *ctx, const struct tm *now, uint8_t *out_digits)
{
    scheduler_slot_t slot;
    uint32_t         remaining;

    if (!ctx->schedule || scheduler_get_next_slot(ctx->schedule, &slot) != ESP_OK)
    {
        return false;
    }

    /*
     * Nothing is made of the armed slot moving any more. It used to be this
     * side's one sign that a treat had been dispensed, and so what put the dog
     * up; the dispenser says so itself now, for every move rather than only the
     * scheduled ones, and says it again once the chime has finished. The slot is
     * still tracked, but only to count down to.
     */
    ctx->slot_shown = slot;
    ctx->slot_known = true;

    remaining = screen_seconds_until(now, &slot);

    out_digits[0] = (uint8_t) (remaining / 3600 / 10);
    out_digits[1] = (uint8_t) (remaining / 3600 % 10);
    out_digits[2] = (uint8_t) (remaining % 3600 / 60 / 10);
    out_digits[3] = (uint8_t) (remaining % 3600 / 60 % 10);
    out_digits[4] = (uint8_t) (remaining % 60 / 10);
    out_digits[5] = (uint8_t) (remaining % 60 % 10);

    return true;
}

/*
 * The slot is a time of day with no date attached, so the next one to come
 * round is either later today or the same time tomorrow. A slot that has just
 * passed rolls to tomorrow; a slot landing on this very second reads as zero
 * rather than as a whole day away.
 */
static uint32_t screen_seconds_until(const struct tm *now, const scheduler_slot_t *slot)
{
    int32_t now_seconds  = now->tm_hour * 3600 + now->tm_min * 60 + now->tm_sec;
    int32_t slot_seconds = slot->hour * 3600 + slot->minute * 60;
    int32_t remaining    = slot_seconds - now_seconds;

    if (remaining < 0)
    {
        remaining += SCREEN_SECONDS_PER_DAY;
    }

    return (uint32_t) remaining;
}

static void screen_fill_digits(uint8_t *digits, size_t count, uint8_t value)
{
    for (size_t i = 0; i < count; i++)
    {
        digits[i] = value;
    }
}

/*
 * A live link beats an open window: if a phone has connected during the pairing
 * minute, that it is talking is the more immediate fact, and the slower pulse is
 * what says so.
 */
static screen_bt_t screen_read_bluetooth(screen_ctx_t *ctx, int64_t now_us)
{
    ble_remote_status_t status;

    if (!ctx->ble || ble_remote_get_status(ctx->ble, &status) != ESP_OK)
    {
        return SCREEN_BT_HIDDEN;
    }

    if (status.connected)
    {
        return screen_blink_on(now_us, SCREEN_BT_CONNECTED_BLINK_MS) ? SCREEN_BT_SHOWN : SCREEN_BT_HIDDEN;
    }

    if (status.pairing_open)
    {
        return screen_blink_on(now_us, SCREEN_BT_PAIRING_BLINK_MS) ? SCREEN_BT_SHOWN : SCREEN_BT_HIDDEN;
    }

    return SCREEN_BT_SHOWN;
}

static bool screen_audio_busy(screen_ctx_t *ctx)
{
    bool playing = false;

    if (!ctx->audio || max98357a_is_playing(ctx->audio, &playing) != ESP_OK)
    {
        return false;
    }

    return playing;
}

/* Phase taken off the clock rather than a counter, so the cadence survives a missed tick */
static bool screen_blink_on(int64_t now_us, uint32_t half_period_ms)
{
    return ((now_us / ((int64_t) half_period_ms * 1000)) % 2) != 0;
}

static bool screen_panel_is_dark(screen_ctx_t *ctx)
{
    gc9a01a_care_status_t care;

    /* Without the care layer running there is nothing that could have darkened the panel behind our back */
    if (gc9a01a_care_get_status(ctx->display, &care) != ESP_OK)
    {
        return false;
    }

    return care.state == GC9A01A_CARE_STATE_BLANKED || care.state == GC9A01A_CARE_STATE_SOAKING;
}

/* One step round the ring, remembering where the face was so that it can be wiped off */
static void screen_advance_drift(screen_ctx_t *ctx)
{
    ctx->wiped_dx = ctx->drift_dx;
    ctx->wiped_dy = ctx->drift_dy;

    ctx->drift_step = (uint8_t) ((ctx->drift_step + 1) % SCREEN_DRIFT_STEPS);

    ctx->drift_dx = screen_drift_offset(ctx->drift_radius_px, ctx->drift_step, 0);
    ctx->drift_dy = screen_drift_offset(ctx->drift_radius_px, ctx->drift_step, SCREEN_DRIFT_STEPS / 4);

    ctx->drifted_us = esp_timer_get_time();
    ctx->moved      = true;
}

static int16_t screen_drift_offset(uint8_t radius, uint8_t step, uint8_t phase)
{
    return (int16_t) (radius * s_drift_sine[(step + phase) % SCREEN_DRIFT_STEPS] / SCREEN_DRIFT_SINE_SCALE);
}

/*
 * Whether every corner of every row is still on the glass at every position the
 * ring visits. Testing the positions themselves rather than the worst corner a
 * radius could reach matters: the ring never puts both offsets at full stretch
 * at once, so assuming it does would give away travel for nothing.
 *
 * Distances are doubled before squaring, which keeps the half pixel centre of a
 * 240 wide panel in integers.
 */
static bool screen_drift_fits(const screen_rect_t *rows, size_t count, uint8_t radius)
{
    int32_t limit = 2 * (GC9A01A_WIDTH / 2 - SCREEN_DRIFT_MARGIN_PX);

    for (uint8_t step = 0; step < SCREEN_DRIFT_STEPS; step++)
    {
        int16_t dx = screen_drift_offset(radius, step, 0);
        int16_t dy = screen_drift_offset(radius, step, SCREEN_DRIFT_STEPS / 4);

        for (size_t i = 0; i < count; i++)
        {
            int32_t left   = (int32_t) rows[i].x + dx;
            int32_t top    = (int32_t) rows[i].y + dy;
            int32_t right  = left + rows[i].width - 1;
            int32_t bottom = top + rows[i].height - 1;

            if (left < 0 || top < 0 || right >= GC9A01A_WIDTH || bottom >= GC9A01A_HEIGHT)
            {
                return false;
            }

            for (int corner = 0; corner < 4; corner++)
            {
                int32_t x = 2 * ((corner & 1) ? right : left) - (GC9A01A_WIDTH - 1);
                int32_t y = 2 * ((corner & 2) ? bottom : top) - (GC9A01A_HEIGHT - 1);

                if (x * x + y * y > limit * limit)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

static uint8_t screen_max_drift_radius(const screen_rect_t *rows, size_t count)
{
    uint8_t radius = 0;

    while (radius < SCREEN_DRIFT_MAX_RADIUS_PX && screen_drift_fits(rows, count, (uint8_t) (radius + 1)))
    {
        radius++;
    }

    return radius;
}
