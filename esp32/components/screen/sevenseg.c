#include "sevenseg.h"

#include <stdbool.h>
#include <stddef.h>

#define SEVENSEG_A 0x01
#define SEVENSEG_B 0x02
#define SEVENSEG_C 0x04
#define SEVENSEG_D 0x08
#define SEVENSEG_E 0x10
#define SEVENSEG_F 0x20
#define SEVENSEG_G 0x40

#define SEVENSEG_SEGMENT_COUNT 7

/*
 *  aaaa      Which segments each value lights. The order below is the usual
 * f    b     one, and the two entries past the digits are the blank cell and
 * f    b     the single middle bar that stands in for a time nothing knows.
 *  gggg
 * e    c
 * e    c
 *  dddd
 */
static const uint8_t s_segments[] = {
    SEVENSEG_A | SEVENSEG_B | SEVENSEG_C | SEVENSEG_D | SEVENSEG_E | SEVENSEG_F,              /* 0 */
    SEVENSEG_B | SEVENSEG_C,                                                                  /* 1 */
    SEVENSEG_A | SEVENSEG_B | SEVENSEG_G | SEVENSEG_E | SEVENSEG_D,                           /* 2 */
    SEVENSEG_A | SEVENSEG_B | SEVENSEG_G | SEVENSEG_C | SEVENSEG_D,                           /* 3 */
    SEVENSEG_F | SEVENSEG_G | SEVENSEG_B | SEVENSEG_C,                                        /* 4 */
    SEVENSEG_A | SEVENSEG_F | SEVENSEG_G | SEVENSEG_C | SEVENSEG_D,                           /* 5 */
    SEVENSEG_A | SEVENSEG_F | SEVENSEG_G | SEVENSEG_E | SEVENSEG_C | SEVENSEG_D,              /* 6 */
    SEVENSEG_A | SEVENSEG_B | SEVENSEG_C,                                                     /* 7 */
    SEVENSEG_A | SEVENSEG_B | SEVENSEG_C | SEVENSEG_D | SEVENSEG_E | SEVENSEG_F | SEVENSEG_G, /* 8 */
    SEVENSEG_A | SEVENSEG_B | SEVENSEG_C | SEVENSEG_D | SEVENSEG_F | SEVENSEG_G,              /* 9 */
    0,                                                                                        /* blank */
    SEVENSEG_G,                                                                               /* dash */
};

static bool sevenseg_style_fits(const sevenseg_style_t *style);

esp_err_t sevenseg_draw_digit(gc9a01a_handle_t display, uint16_t x, uint16_t y, const sevenseg_style_t *style,
                              uint8_t value, uint16_t on_color, uint16_t off_color)
{
    uint16_t stroke;
    uint16_t bar;
    uint16_t arm;
    uint8_t  lit;

    if (!display || !sevenseg_style_fits(style) || value >= sizeof(s_segments) / sizeof(s_segments[0]))
    {
        return ESP_ERR_INVALID_ARG;
    }

    stroke = style->stroke;
    bar    = style->width - 2 * stroke;        /* length of the three horizontal segments */
    arm    = (style->height - 3 * stroke) / 2; /* length of the four vertical ones */
    lit    = s_segments[value];

    const struct
    {
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        uint8_t  bit;
    } segments[SEVENSEG_SEGMENT_COUNT] = {
        {x + stroke, y, bar, stroke, SEVENSEG_A},
        {x + style->width - stroke, y + stroke, stroke, arm, SEVENSEG_B},
        {x + style->width - stroke, y + 2 * stroke + arm, stroke, arm, SEVENSEG_C},
        {x + stroke, y + style->height - stroke, bar, stroke, SEVENSEG_D},
        {x, y + 2 * stroke + arm, stroke, arm, SEVENSEG_E},
        {x, y + stroke, stroke, arm, SEVENSEG_F},
        {x + stroke, y + stroke + arm, bar, stroke, SEVENSEG_G},
    };

    for (size_t i = 0; i < SEVENSEG_SEGMENT_COUNT; i++)
    {
        esp_err_t err = gc9a01a_fill_rect(display, segments[i].x, segments[i].y, segments[i].width, segments[i].height,
                                          (lit & segments[i].bit) ? on_color : off_color);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

esp_err_t sevenseg_draw_colon(gc9a01a_handle_t display, uint16_t x, uint16_t y, const sevenseg_style_t *style,
                              uint16_t color)
{
    esp_err_t err;
    uint16_t  stroke;

    if (!display || !sevenseg_style_fits(style))
    {
        return ESP_ERR_INVALID_ARG;
    }

    stroke = style->stroke;

    err = gc9a01a_fill_rect(display, x, y + style->height / 3 - stroke / 2, stroke, stroke, color);
    if (err != ESP_OK)
    {
        return err;
    }

    return gc9a01a_fill_rect(display, x, y + 2 * style->height / 3 - stroke / 2, stroke, stroke, color);
}

uint16_t sevenseg_colon_width(const sevenseg_style_t *style)
{
    return style ? style->stroke : 0;
}

/*
 * The cell has to be wide enough for two vertical segments with something
 * between them, and tall enough for the three horizontal ones with an arm above
 * and below the middle. The arm length is halved, so an odd leftover would put
 * the middle bar off centre by a pixel; requiring an even remainder keeps the
 * digit symmetric.
 */
static bool sevenseg_style_fits(const sevenseg_style_t *style)
{
    if (!style || style->stroke == 0)
    {
        return false;
    }

    if (style->width < 2 * style->stroke + 1 || style->height < 3 * style->stroke + 2)
    {
        return false;
    }

    return ((style->height - 3 * style->stroke) % 2) == 0;
}
