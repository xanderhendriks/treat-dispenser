#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "gc9a01a.h"

/*
 * A three letter font, which is all the face needs beyond digits.
 *
 * The seven segment renderer cannot help here: A and P are fine on seven
 * segments but M is the classic one that is not, there being no way to draw its
 * two inner diagonals out of seven bars. Rather than fake it, AM and PM get a
 * proper 5 by 7 pixel font, scaled up by whole pixels so the edges stay on the
 * grid and it sits beside the digits without looking like a different device.
 *
 * Whole pixel scaling rather than a real typeface is also why there is no font
 * file to carry: at the size this label wants, an antialiased glyph would only
 * come out mushy.
 */

#define PIXELFONT_CELL_WIDTH  5
#define PIXELFONT_CELL_HEIGHT 7

/**
 * Draw a short string, scaled up by a whole number of pixels.
 *
 * Both the lit and the unlit pixels are painted, so redrawing in place changes
 * the text without having to clear behind it first, the same way the digits
 * work. Pass the background as off_color for plain text.
 *
 * Only 'A', 'M' and 'P' exist, which is what AM and PM are made of; anything
 * else is refused rather than quietly dropped.
 *
 * @param display Panel handle.
 * @param x Left edge.
 * @param y Top edge.
 * @param scale Pixels per font pixel, at least 1.
 * @param text String to draw.
 * @param on_color Colour for lit pixels.
 * @param off_color Colour for the rest of the cell.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a scale of 0 or a
 *         character the font does not have.
 */
esp_err_t pixelfont_draw(gc9a01a_handle_t display, uint16_t x, uint16_t y, uint8_t scale, const char *text,
                         uint16_t on_color, uint16_t off_color);

/**
 * Width the given string occupies, for laying a row out.
 *
 * @param scale Pixels per font pixel.
 * @param text String to measure.
 * @return Width in pixels, including the one font pixel between letters.
 */
uint16_t pixelfont_width(uint8_t scale, const char *text);

/**
 * Height of a line at the given scale.
 *
 * @param scale Pixels per font pixel.
 * @return Height in pixels.
 */
uint16_t pixelfont_height(uint8_t scale);
