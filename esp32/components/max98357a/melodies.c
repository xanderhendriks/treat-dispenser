#include "melodies.h"

/* Note frequencies in Hz */
#define NOTE_G4  392
#define NOTE_AS4 466 /* Bb4 */
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_DS5 622 /* Eb5 */
#define NOTE_E5  659
#define NOTE_G5  784

/* Base durations at 92 BPM (quarter note = 652 ms) */
#define EIGHTH         326
#define QUARTER        652
#define DOTTED_QUARTER 978
#define HALF           1304

/*
 * Intro hook of "The Longest Time" (Billy Joel), key of Eb major, notated an
 * octave above the lead vocal so it carries on a small speaker. Pitches and
 * rhythm verified against the published sheet music and a MIDI transcription:
 * "Whoa oh oh oh" = Eb D Eb C, "for the long-est time" = D C D C Bb, and the
 * final "time" resolves up to Eb.
 */
static const melody_note_t s_for_the_longest_time_notes[] = {
    {NOTE_DS5, HALF},           /* Whoa    */
    {NOTE_D5, QUARTER},         /* oh      */
    {NOTE_DS5, QUARTER},        /* oh      */
    {NOTE_C5, DOTTED_QUARTER},  /* oh      */
    {0, EIGHTH},                /* (rest)  */
    {NOTE_D5, EIGHTH},          /* for     */
    {NOTE_C5, EIGHTH},          /* the     */
    {NOTE_D5, EIGHTH},          /* long    */
    {NOTE_C5, EIGHTH},          /* est     */
    {NOTE_AS4, DOTTED_QUARTER}, /* time    */
    {0, EIGHTH},                /* (rest)  */
    {NOTE_D5, QUARTER},         /* Whoa    */
    {NOTE_DS5, QUARTER},        /* oh      */
    {NOTE_C5, DOTTED_QUARTER},  /* oh      */
    {0, EIGHTH},                /* (rest)  */
    {NOTE_D5, EIGHTH},          /* for     */
    {NOTE_C5, EIGHTH},          /* the     */
    {NOTE_D5, EIGHTH},          /* long    */
    {NOTE_C5, EIGHTH},          /* est     */
    {NOTE_DS5, HALF + QUARTER}, /* time    */
    {0, HALF},                  /* (rest)  */
};

/*
 * A tada flourish: a quick run up a C major triad onto a held fifth. Short
 * enough to sit under a second, which is all the acknowledgement that reaching
 * the home position needs.
 */
static const melody_note_t s_tada_notes[] = {
    {NOTE_G4, 110}, /* ta   */
    {NOTE_C5, 110}, /*      */
    {NOTE_E5, 110}, /*      */
    {NOTE_G5, 620}, /* daa  */
    {0, 120},       /* (rest) */
};

static const melody_t s_melodies[MAX98357A_MELODY_COUNT] = {
    [MAX98357A_MELODY_FOR_THE_LONGEST_TIME] =
        {
            .notes      = s_for_the_longest_time_notes,
            .note_count = sizeof(s_for_the_longest_time_notes) / sizeof(s_for_the_longest_time_notes[0]),
        },
    [MAX98357A_MELODY_TADA] =
        {
            .notes      = s_tada_notes,
            .note_count = sizeof(s_tada_notes) / sizeof(s_tada_notes[0]),
        },
};

const melody_t *melody_get(max98357a_melody_t melody)
{
    if (melody >= MAX98357A_MELODY_COUNT)
    {
        return NULL;
    }

    return &s_melodies[melody];
}
