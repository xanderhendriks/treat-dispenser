#include "pixelfont.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* One font pixel of air between letters, so AM does not run together */
#define PIXELFONT_ADVANCE (PIXELFONT_CELL_WIDTH + 1)

/* Bit 4 is the leftmost column of the five */
#define PIXELFONT_LEFT_BIT 0x10

/*
 *  .###.    #...#    ####.
 *  #...#    ##.##    #...#
 *  #...#    #.#.#    #...#
 *  #####    #.#.#    ####.
 *  #...#    #...#    #....
 *  #...#    #...#    #....
 *  #...#    #...#    #....
 */
static const uint8_t s_glyph_a[PIXELFONT_CELL_HEIGHT] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
static const uint8_t s_glyph_m[PIXELFONT_CELL_HEIGHT] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
static const uint8_t s_glyph_p[PIXELFONT_CELL_HEIGHT] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};

static const uint8_t *pixelfont_glyph(char c);

esp_err_t pixelfont_draw(gc9a01a_handle_t display, uint16_t x, uint16_t y, uint8_t scale, const char *text,
                         uint16_t on_color, uint16_t off_color)
{
    if (!display || !text || scale == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (const char *c = text; *c; c++)
    {
        const uint8_t *glyph = pixelfont_glyph(*c);

        if (!glyph)
        {
            return ESP_ERR_INVALID_ARG;
        }

        for (uint8_t row = 0; row < PIXELFONT_CELL_HEIGHT; row++)
        {
            uint8_t column = 0;

            /*
             * Runs of like pixels go out as one rectangle rather than one per
             * pixel, which turns a glyph from thirty five writes into about a
             * dozen.
             */
            while (column < PIXELFONT_CELL_WIDTH)
            {
                bool    lit = (glyph[row] & (PIXELFONT_LEFT_BIT >> column)) != 0;
                uint8_t run = 1;

                while (column + run < PIXELFONT_CELL_WIDTH &&
                       (((glyph[row] & (PIXELFONT_LEFT_BIT >> (column + run))) != 0) == lit))
                {
                    run++;
                }

                esp_err_t err =
                    gc9a01a_fill_rect(display, (uint16_t) (x + column * scale), (uint16_t) (y + row * scale),
                                      (uint16_t) (run * scale), scale, lit ? on_color : off_color);
                if (err != ESP_OK)
                {
                    return err;
                }

                column = (uint8_t) (column + run);
            }
        }

        x = (uint16_t) (x + PIXELFONT_ADVANCE * scale);
    }

    return ESP_OK;
}

uint16_t pixelfont_width(uint8_t scale, const char *text)
{
    size_t length = text ? strlen(text) : 0;

    if (length == 0)
    {
        return 0;
    }

    /* Every letter but the last carries its trailing pixel of air */
    return (uint16_t) (scale * (length * PIXELFONT_ADVANCE - 1));
}

uint16_t pixelfont_height(uint8_t scale)
{
    return (uint16_t) (scale * PIXELFONT_CELL_HEIGHT);
}

static const uint8_t *pixelfont_glyph(char c)
{
    switch (c)
    {
        case 'A':
            return s_glyph_a;
        case 'M':
            return s_glyph_m;
        case 'P':
            return s_glyph_p;
        default:
            return NULL;
    }
}
