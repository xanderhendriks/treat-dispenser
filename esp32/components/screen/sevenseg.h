#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "gc9a01a.h"

/*
 * Seven segment digits drawn out of rectangles rather than out of a font.
 *
 * A bitmap font would mean carrying glyph tables for two sizes and would still
 * only ever be asked for the ten digits and a colon. Seven rectangles per digit
 * gives any size the layout wants, stays crisp because every edge lands on a
 * pixel boundary, and reads well on a round panel.
 *
 * It also makes updates cheap, which matters for a display that is redrawn
 * every second: each segment is painted in either the lit or the unlit colour,
 * so a digit can be changed in place without first clearing its cell and
 * without the flicker that would come with that. The gaps between the segments
 * are never touched, so they keep whatever the background was painted with.
 */

/** Nothing lit, for a digit position that has no value to show. */
#define SEVENSEG_BLANK 10

/** The middle bar alone, which is what stands in for an unknown time. */
#define SEVENSEG_DASH 11

typedef struct
{
    uint16_t width;  /* cell width, at least 2 * stroke + 1 */
    uint16_t height; /* cell height, at least 3 * stroke + 2 */
    uint16_t stroke; /* segment thickness */
} sevenseg_style_t;

/**
 * Draw one digit in place.
 *
 * Every one of the seven segments is painted, lit ones in on_color and the rest
 * in off_color, so calling this again with a different value leaves nothing of
 * the old one behind. Pass the background as off_color for plain digits.
 *
 * @param display Panel handle.
 * @param x Left edge of the digit cell.
 * @param y Top edge of the digit cell.
 * @param style Cell size and stroke.
 * @param value 0-9, SEVENSEG_BLANK or SEVENSEG_DASH.
 * @param on_color Colour for lit segments.
 * @param off_color Colour for unlit segments.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a value out of range or a
 *         style whose stroke does not fit its cell.
 */
esp_err_t sevenseg_draw_digit(gc9a01a_handle_t display, uint16_t x, uint16_t y, const sevenseg_style_t *style,
                              uint8_t value, uint16_t on_color, uint16_t off_color);

/**
 * Draw a colon separator sized to match a digit cell.
 *
 * Two square dots, set a third and two thirds of the way down the cell so they
 * straddle the middle segment of the digits either side.
 *
 * @param display Panel handle.
 * @param x Left edge of the colon cell.
 * @param y Top edge, the same as the digits it sits between.
 * @param style The style of those digits.
 * @param color Dot colour.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for an unusable style.
 */
esp_err_t sevenseg_draw_colon(gc9a01a_handle_t display, uint16_t x, uint16_t y, const sevenseg_style_t *style,
                              uint16_t color);

/**
 * Width a colon occupies in the given style, for laying a row out.
 *
 * @param style Digit style the colon sits among.
 * @return Width in pixels.
 */
uint16_t sevenseg_colon_width(const sevenseg_style_t *style);
