#!/usr/bin/env python3
"""Draw the Bluetooth badge, as the master for the panel's status icon.

    draw_bluetooth.py components/screen/assets/bt_source.png

A white rune on a rounded blue field, which is the form the logo is usually met
in and the one that reads at a glance: a bare rune on black is a few thin
strokes, while a filled badge is a shape the eye catches without looking for it.

The rune is drawn by construction rather than traced, because it is all right
angles once you see it. Put the two apexes a quarter of the rune's height down
from the top and up from the bottom, and every stroke falls out at exactly 45
degrees: the run from the top of the stem to the upper apex and the run from that
apex back to the centre are then the same length, which is what closes the
triangle cleanly. The left arms end level with the apexes, as the logo has them.

Two places this departs from the logo, both because the target is forty pixels
tall rather than a thousand:

  - the stroke is a little over a twelfth of the rune's height rather than the
    logo's own twentieth, which at this size would come out under two pixels and
    read as scratchy,
  - the field is flat rather than the gradient and gloss of the app icon, which
    would be invisible at this size and would only give the RGB565 dither
    something to chew on.

Needs Pillow, the same as tools/img2rgb565.py; see that script for the venv.
"""

import sys

try:
    from PIL import Image, ImageDraw
except ImportError:
    sys.exit('Pillow is missing; see tools/img2rgb565.py for the venv incantation.')

CANVAS = 1000
BADGE_ASPECT = 0.72     # width over height, as the app icon has it
CORNER_RATIO = 0.42     # of the badge width: rounded almost to a stadium, which is the look
RUNE_FILL = 0.84        # how much of the badge height the rune takes
STROKE_RATIO = 0.08     # of the rune height, against the logo's own 0.048

BLUETOOTH_BLUE = (0, 130, 252, 255)
RUNE_WHITE = (255, 255, 255, 255)


def draw(path):
    badge_height = int(CANVAS * 0.90)
    badge_width = int(badge_height * BADGE_ASPECT)
    left = (CANVAS - badge_width) // 2
    top = (CANVAS - badge_height) // 2

    image = Image.new('RGBA', (CANVAS, CANVAS), (0, 0, 0, 0))
    pen = ImageDraw.Draw(image)

    pen.rounded_rectangle([left, top, left + badge_width, top + badge_height],
                          radius=int(badge_width * CORNER_RATIO), fill=BLUETOOTH_BLUE)

    rune_height = int(badge_height * RUNE_FILL)
    rune_top = top + (badge_height - rune_height) // 2
    rune_bottom = rune_top + rune_height
    centre = rune_top + rune_height // 2
    stem = left + badge_width // 2
    reach = rune_height // 4        # how far out the apexes and the arm ends sit
    stroke = int(rune_height * STROKE_RATIO)

    for points in (
        [(stem, rune_top), (stem, rune_bottom)],                                    # stem
        [(stem, rune_top), (stem + reach, rune_top + reach), (stem, centre)],       # upper triangle
        [(stem, centre), (stem + reach, rune_bottom - reach), (stem, rune_bottom)],  # lower triangle
        [(stem - reach, rune_top + reach), (stem, centre)],                         # upper left arm
        [(stem - reach, rune_bottom - reach), (stem, centre)],                      # lower left arm
    ):
        pen.line(points, fill=RUNE_WHITE, width=stroke, joint='curve')

    image.save(path)

    bbox = image.split()[3].point(lambda v: 255 if v > 16 else 0).getbbox()
    width, height = bbox[2] - bbox[0], bbox[3] - bbox[1]
    print('%s: content %dx%d, aspect %.3f' % (path, width, height, width / height))
    for target in (34, 36, 40):
        print('  at %d tall, %d wide' % (target, round(target * width / height)))


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('usage: draw_bluetooth.py <output.png>')
    draw(sys.argv[1])
