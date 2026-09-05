#pragma once

#include <stddef.h>
#include <stdint.h>

#include "max98357a.h"

typedef struct
{
    uint16_t frequency_hz; /* 0 for a rest */
    uint16_t duration_ms;
} melody_note_t;

typedef struct
{
    const melody_note_t *notes;
    size_t               note_count;
} melody_t;

/**
 * Look up the note table for a melody.
 *
 * @param melody Melody identifier.
 * @return The melody or NULL for an unknown identifier.
 */
const melody_t *melody_get(max98357a_melody_t melody);
