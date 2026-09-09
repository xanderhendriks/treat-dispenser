#pragma once

#include "max98357a.h"

/*
 * A tada marks arriving at the home position, and the Billy Joel hook plays
 * twice once a treat has been dispensed.
 *
 * The tada doubles as the short acknowledgement: it is what a move backwards
 * plays, having dispensed nothing worth singing about, and what any move driven
 * by hand plays, whether from the phone or the console, since the app screen or
 * the printed result has already said the move finished. Only a treat the
 * dispenser gives out on its own account earns the full tune.
 */
#define HOME_MELODY        MAX98357A_MELODY_TADA
#define HOME_MELODY_TIMES  1
#define TREAT_MELODY       MAX98357A_MELODY_FOR_THE_LONGEST_TIME
#define TREAT_MELODY_TIMES 2
#define SHORT_MELODY       MAX98357A_MELODY_TADA
#define SHORT_MELODY_TIMES 1
#define MELODY_VOLUME_PCT  75
