Treat Dispenser
===============

Firmware for the treat dispenser board (ESP32-S3-WROOM-1-N16). The board
drives a DC motor through a TI DRV8871 H-bridge (IN1 on GPIO4, IN2 on GPIO5)
and a speaker through a MAX98357A I2S amplifier (BCLK on GPIO16, LRCLK on
GPIO17, DIN on GPIO18, SD_MODE on GPIO21). An RV-3028-C7 RTC (0x52) and the
CH224A USB-PD sink controller (0x22 or 0x23) share an I2C bus on SDA GPIO6 and
SCL GPIO7; the RTC drives its open drain INT pin into GPIO15 against a 4k7
pull-up. A 100k/27k divider (R5/R8) brings the +9V rail down to GPIO2 so the
firmware can measure it. An ER-TFTM1.28-1 round 240x240 display hangs off
SPI2 (CS GPIO9, DC GPIO10, MOSI GPIO11, SCK GPIO12, RES GPIO13, backlight
GPIO14).
Developed with ESP-IDF v5.5.1.

The CH224A is strapped for single resistor configuration with a 6.8k resistor
from CFG1 to GND, which requests 9V and enables its I2C interface. That output
is the board's only supply rail and D1 (SMBJ12A) clamps it, so the firmware
refuses to request anything above 9V.

Development environment
-----------------------

Open the repository in VS Code and reopen it in the dev container
(``.devcontainer/devcontainer.json`` builds the ``Dockerfile``). The container
provides ESP-IDF, clang-format and the SEGGER J-Link tools.

Building
--------

From the ``esp32`` directory inside the container::

    idf.py build

Flashing and monitoring
-----------------------

The board is normally flashed through the J-Link on the STDC14 debug adapter
(J6), using the custom ``jtag-flash`` action defined in ``esp32/idf_ext.py``.
JTAG flashing needs no button presses and works regardless of what the chip is
currently running::

    idf.py jtag-flash
    idf.py -p /dev/ttyUSB0 monitor

``jtag-flash`` accepts ``--app-only`` to flash just the application (skipping
bootloader and partition table) and ``--jtag-speed <kHz>`` (default 4000). The
monitor runs over the FTDI cable on J6 pins 1/2.

Both the J-Link and the FTDI cable enumerate on Windows; attach them to WSL
with ``usbipd attach --wsl --busid <id>`` (list with ``usbipd list``) before
using them inside the container.

One-time eFuse setup
~~~~~~~~~~~~~~~~~~~~

A fresh ESP32-S3 routes JTAG to its internal USB-Serial-JTAG peripheral, and
this board does not wire out the USB data lines, so external JTAG is dead
until the ``DIS_USB_JTAG`` eFuse is burned. This is permanent and must be done
once per board. Put the chip in download mode (hold BOOT, tap RESET, release
BOOT) and run::

    python -m espefuse --port /dev/ttyUSB0 burn_efuse DIS_USB_JTAG

Verify that ``DIS_USB_JTAG = True`` afterwards::

    python -m espefuse --port /dev/ttyUSB0 summary

Serial flashing (backup)
~~~~~~~~~~~~~~~~~~~~~~~~

Flashing over the UART works without the J-Link and is unaffected by the
eFuse, but the FTDI cable has no DTR/RTS auto-reset wiring, so the ROM
bootloader must be entered by hand: hold BOOT, tap RESET, release BOOT,
then::

    idf.py -p /dev/ttyUSB0 flash monitor

Because esptool cannot reset the board either, tap RESET once more after
flashing to boot the new firmware.

Drum positioning
----------------

The drum carries one magnet per treat slot, and the DRV5055 Hall sensor sees
each of them go by. The magnet at the home position is mounted the other way
round, so it is the only one that reads as a negative field; every other magnet
reads positive. ``home`` uses that to find a known starting position, driving
past normal magnets until the negative one shows up, and ``next`` steps the
drum on by exactly one slot regardless of polarity.

Both moves brake the motor as soon as a magnet is detected and then release the
brake, and both give up with an error if no magnet turns up within five
seconds, which is how a jammed drum reports itself.

Each move ends with a sound: reaching home plays a short tada, and reaching the
next position plays the hook from The Longest Time twice, which is roughly how
long it takes a dog to work out that a treat has arrived. Playback runs in the
background, so it does not hold the move up.

Feeding schedule
----------------

Treats are dispensed automatically at 07:00, 15:00 and 23:00. The RV-3028 alarm
matches on the hour and minute with the day masked off, so it can only hold one
time at once; the firmware arms it for the next feeding time, and re-arms it for
the one after that every time it fires. Keeping the alarm pinned to wall clock
times this way means the schedule cannot drift the way a repeating interval
would.

The alarm pulls the RTC INT pin low, which wakes a task that runs one ``next``
move and then re-arms. Clearing the alarm flag is what releases INT again, so
that happens on every path, including a failed move. The task also re-reads the
alarm flag once a minute as a safety net, which covers the one case an edge
cannot: a flag that was already set, and INT therefore already low, before the
interrupt was hooked up.

The schedule is computed from the RTC, so ``time_set`` re-arms the alarm for
whatever the next feeding time is under the new clock. Without a working RTC the
firmware still runs, but only ``home`` and ``next`` from the console.

Phone link
----------

The dispenser is driven from an Android phone over BLE; the app lives in
``app-treat-dispenser``. The ESP32-S3 has no Bluetooth Classic radio, so BLE
GATT is the only option, and NimBLE is used rather than Bluedroid because it
costs about half the flash and RAM for a peripheral this small.

WiFi is never brought up — nothing calls ``esp_wifi_init`` — and
``CONFIG_ESP_COEX_SW_COEXIST_ENABLE`` is off, which hands the whole radio to
BLE. ``CONFIG_ESP_WIFI_ENABLED`` cannot be turned off in ``sdkconfig``: it is a
hidden symbol that only tracks what the silicon can do.

One custom service carries two characteristics:

- command, write only, one byte: ``0x01`` runs ``home``, ``0x02`` runs ``next``
- status, read and notify, four bytes: state (0 idle, 1 busy), the last command,
  how it ended (1 ok, 2 timeout, 3 home magnet not found, 4 failed) and whether
  the drum is parked on the home magnet

  ==========  ======================================
  service     ffc50e4e-afd5-4edd-86a1-b41c94120001
  command     ffc50e4e-afd5-4edd-86a1-b41c94120002
  status      ffc50e4e-afd5-4edd-86a1-b41c94120003
  ==========  ======================================

A move takes seconds, so a write is acknowledged immediately and carried out by
a worker task; the app watches the status notification to see it finish.

Pairing
~~~~~~~

Both characteristics require an encrypted link, so an unpaired phone can find
the dispenser and discover the service but cannot turn the drum. Pairing uses
LE Secure Connections with Just Works, the board having neither a display nor a
keypad to show a passkey on.

Just Works can be intercepted, but only during the pairing exchange itself, so
that exchange is kept to a **60 second window from start-up**. After the window
shuts, a phone that is not already bonded is disconnected as soon as it
connects, and a request to pair again is ignored. Bonds live in NVS, so a paired
phone reconnects silently for as long as it is bonded, whatever the window is
doing.

To adopt another phone, either reset the board or run ``ble_pair`` on the
console. ``ble_forget`` deletes every bond; the phone keeps its own half, so it
has to forget the dispenser in the Android Bluetooth settings before it can pair
again.

Over the air updates
~~~~~~~~~~~~~~~~~~~~

The partition table already carries two 8 MB OTA slots, and nothing yet writes
to them. Over BLE the transfer is limited by the link rather than the flash:
with the 2M PHY, data length extension, a 517 byte ATT MTU, a 15 ms connection
interval and write-without-response, 20-60 kB/s is what a phone actually
manages, so the current 690 kB image would take well under a minute. Using
write-with-response for the payload drops that to one packet per connection
event, which turns the same transfer into minutes; that is the mistake to avoid.

Display
-------

The board carries an ER-TFTM1.28-1: a 1.28 inch round 240x240 IPS panel on a
breakout board, driven by a GC9A01A over 4-wire SPI. CS is GPIO9, DC GPIO10,
MOSI GPIO11, SCK GPIO12, RES GPIO13 and BLK GPIO14. MOSI and SCK land on the
ESP32-S3 IOMUX pins for SPI2, so the clock is not held down to what the GPIO
matrix can pass; R14 and R15 put 33R in series with both. The driver runs at
40 MHz, which is what the vendor sample code for the module uses, and the
controller's own 10 ns write cycle leaves headroom above that if the wiring
turns out to take it.

The controller always presents a square 240x240 frame memory but the glass only
shows the circle inscribed in it, so the four corners of every buffer are
written and never seen. The eight pin header brings out neither the tearing
effect pin nor a separate data output, so the driver never waits on TE and never
reads a register back: the frame is written blind, and the only confirmation
that the link works is what appears on the glass. ``display_test`` draws a
pattern for exactly that, with a grey wedge, a hue wedge and four coloured tabs
that say which edge of the frame is which.

Register setup is transcribed from the module's own sample code, most of it
undocumented gate timing, gamma and power settings for this particular glass.
The backlight is PWM on BLK from LEDC timer 1 channel 2, keeping clear of
timer 0 and channels 0 and 1 that the motor driver holds; the controller's own
LEDPWM output, which the brightness registers drive, is not on the header.

Clock face
~~~~~~~~~~

The display shows three rows, white on black, with no frame, border or divider
anywhere: the dog at the top, how long until the next treat in large digits
across the middle, and the wall clock small at the bottom.

The digits are seven segment, drawn out of rectangles rather than from a font:
the face only ever needs ten digits and a colon, and seven rectangles per digit
gives any size the layout asks for, keeps every edge on a pixel boundary, and
lets a digit be changed in place without first clearing its cell and flickering.

The clock is twelve hour with AM or PM, this being for an Australian dog.
Midnight and noon both read as twelve and the leading hour digit is blanked
rather than zeroed before ten, which is what ``SEVENSEG_BLANK`` is for; the
digit keeps its slot, so the time does not shift sideways as the hour crosses
ten, at the cost of looking a few pixels right of centre for most of the day.

The label itself cannot be seven segment, and not for want of trying: A and P
are both fine on seven bars but M is the classic glyph that is not, having two
inner diagonals and nowhere to put them. So AM and PM come from ``pixelfont.c``,
a three letter 5 by 7 font scaled up by whole pixels. Whole pixels rather than a
real typeface because at this size an antialiased glyph would only come out
mushy, and three letters because that is all that is needed — anything else is
refused rather than quietly dropped. Runs of like pixels go out as one rectangle,
which turns a glyph from thirty five writes into about a dozen.

The label sits on the same bottom edge as the digits rather than centred against
them. Centred, a label two thirds their height floats above the line they stand
on and reads as having come adrift; a shared baseline is what makes the time and
its AM or PM look like one row.

Rotation 90 is what the assembled enclosure needs. The three rows are 67 by 88
for the dog, 175 by 41 for the countdown and 94 by 23 for the clock and its
label, with eighteen pixels between them, and the stack of them is centred as a
whole: the dog lands at (86,26), the countdown at y 132 to 172 and the clock at
y 191 to 213, leaving twenty six pixels of air top and bottom.

Centring the stack rather than pinning the countdown to the middle of the panel
is deliberate. The dog is three times the height of the clock row, so holding
the countdown at the centre crowds the top of the dog against the bezel and
leaves a large empty arc under the time, and the whole face reads as having slid
upwards. The countdown is still the middle row and still what the eye goes to,
which is what being in the middle was for.

The face does not stay there. Once a minute it steps to the next of eight
positions round a ring twelve pixels out from the middle, and one full circuit
takes eight minutes, so no position is occupied more than an eighth of the time.
The step size is the whole point: consecutive positions are eight and a half
pixels apart, against a five pixel stroke, so a segment lands clear of where it
was rather than mostly on top of itself. The screen care layer's own two pixel
nudge is still there underneath and composes with this, but on its own it was
never going to do anything for a stroke that wide.

``screen_max_drift_radius()`` walks the radius up a pixel at a time and stops
where a corner would leave the glass, so the layout decides what is affordable
and growing something on the face costs travel rather than pushing it under the
bezel. It measures the three rows individually rather than their bounding box,
and the difference is not small: the countdown is 175 wide and the other two
about half that, so a bounding box charges the dog and the clock for corners
that hold nothing, at the top and bottom of a circle where the room runs out.
For this layout the box allows four pixels of travel and the rows allow twelve.
It also tests the eight positions the ring actually visits instead of the worst
corner the radius could reach, since the ring never puts both offsets at full
stretch at once.

Twelve is where the sizes were set from, rather than the other way round. A
nineteen pixel hop once a minute reads as the face jumping rather than drifting,
so twelve is the figure worth keeping, and the dog and the gaps were then grown
until the layout allowed exactly that and no more. The furthest drawn corner
ends up 116 of 120 from the centre. Anything further — a taller dog, wider gaps —
comes straight out of the travel, and ``screen_max_drift_radius()`` will clamp
and log rather than let it slide under the bezel.

Moving the face means wiping where it was, which is done as the one bounding
box rather than the whole panel. Clearing all 240 by 240 would be about twice
the pixels and would read as a flicker once a minute; this way the gap between
the old face going and the new one arriving is a few milliseconds, inside a
frame. ``display`` reports where the face is currently sitting.

Bluetooth icon
~~~~~~~~~~~~~~

The Bluetooth badge sits in the empty ground to the left of the dog, level with
the middle of his head, at 26 by 36: a white rune on a rounded blue field, which
is the form the logo is usually met in and the one that reads at a glance. A
bare rune on black is a few thin strokes; a filled badge is a shape the eye
catches without looking for it. It is one mark, so all a state has to say is
whether it is on the glass:

- blinking briskly, half a second on and half off, while the pairing window is
  open and a new phone may bond,
- pulsing at half that rate while a phone is actually connected,
- steady when neither, which is the dispenser saying it talks Bluetooth and is
  listening to phones it already knows,
- gone when the radio never came up. An icon that is there whether or not the
  thing works says nothing, so the useful state is drawing nothing at all.

The brisk pulse goes to pairing rather than to the connection because pairing
is the state that wants noticing and the only one with a deadline; a connected
phone is a calmer fact. A live link takes precedence over an open window, so a
phone that bonds during the pairing minute slows the badge rather than leaving
it hurrying.

Neither rate can go much quicker than the brisk one. The face is painted four
times a second, so a half period shorter than two ticks would be sampled at
about its own rate and beat against it, and the blink would stutter instead of
keeping time; five hundred milliseconds is that floor. The phase is taken off
the clock rather than off a counter, so a missed tick shortens one flash rather
than shifting the cadence.

``tools/draw_bluetooth.py`` draws the master, by construction rather than by
tracing, because the rune is all right angles once you see it: put the two
apexes a quarter of the rune's height down from the top and up from the bottom,
and every stroke falls out at exactly 45 degrees. The run from the top of the
stem to the upper apex and the run from that apex back to the centre are then
the same length, which is what closes the triangle cleanly, and the left arms
end level with the apexes. It goes through the same converter as the dog::

    ~/.venvs/imgtools/bin/python tools/draw_bluetooth.py \
        components/screen/assets/bt_source.png
    ~/.venvs/imgtools/bin/python tools/img2rgb565.py \
        components/screen/assets/bt_source.png \
        --trim --size 26x36 --name bt_icon --out components/screen/assets

Two places it departs from the logo, both because the target is thirty six
pixels tall rather than a thousand. The stroke is a little over a twelfth of the
rune's height rather than the logo's own twentieth, which at this size would
come out under two pixels and read as scratchy; and the field is flat rather
than the gradient and gloss of the app icon, which would be invisible this small
and would only give the RGB565 dither something to chew on. Drawing it large and
scaling down rather than plotting it pixel by pixel is what gets the diagonals
and the rounded corners antialiased instead of stair-stepped.

Being a status mark rather than part of the composition, the dog stays centred
and the icon takes ground that was doing nothing. It is registered as a drawn
rectangle like the three rows, so the drift envelope accounts for it, and it
happens to sit where the circle is at its widest: travel is 12 with it and 12
without, so it costs nothing at all. The badge being wider than the bare rune
was does push the furthest drawn corner from 115.7 out to 117.0 of 120, which is
the margin it eats rather than travel.

The radio is started before the screen for this, the screen reading
``ble_remote_get_status()`` on each tick.

The dog
~~~~~~~

There are two portraits, and the difference between them is entirely about
image sticking. The 67 by 88 badge at the top is permanent, so it has to be
small and it has to ride the drift with everything else. The 144 by 188 one is
shown only when a treat drops: it takes the whole panel for five seconds and
then the face comes back, and because it is over in seconds it carries no
sticking risk at all, which is what earns it the size and the colour.

Neither is square, and that is the point. His head is 944 by 1238 of drawing,
about three parts wide to four tall, so a square asset is a quarter empty before
anything is drawn in it. Nothing on this panel cares: ``gc9a01a_draw_bitmap()``
takes a width and a height, ``screen_layout()`` reads both out of the generated
header, and the celebration centres whatever it is given. So both assets are cut
to his shape instead, which spends every pixel on him and needs no distortion to
do it.

For the large one that is worth a good deal. The biggest square inside a 240
circle is 168, and he only filled 125 by 165 of it; the biggest rectangle of his
own proportions is 145 by 190, and 144 by 188 is that with a pixel of margin
kept back. Same picture, fifteen per cent bigger on the glass, and slightly
fewer bytes.

The badge has grown twice, from 43 by 56 to 55 by 72 with the move to three
rows and then to 67 by 88. The first of those reads as costing travel and did
not: measuring the rows instead of their bounding box found so much more room
than the old model admitted to that the dog got thirty per cent bigger and the
travel went up as well. The second spent what was left, taking the layout from
nineteen pixels of available travel down to the twelve actually wanted.

He goes up whenever the drum turns, and stays up until the chime it ends on has
finished, so how long that is comes from the move and the melody rather than
from a timer. A treat dispensed on the schedule earns the full tune and so keeps
him on screen for it; a move driven by hand gets the short tada and hands the
panel back sooner.

The dispenser is what says so, through ``dispenser_set_move_observer()``, rather
than each caller remembering to. Every move anything asks for goes through
``dispenser_move()`` or ``dispenser_go_to_slot()`` — the schedule, the phone and
the console all arrive there — so one observer around those two covers the lot,
and nothing can be added later that quietly misses it. The observer fires on
both paths out, a jam included, so a failed move cannot leave him stranded.

Waiting for the sound is ``SCREEN_CELEBRATE_UNTIL_QUIET``: the observer asks for
a hold as the drum starts and for that as it stops, and the screen then watches
``max98357a_is_playing()``. There is no race in it, because ``max98357a_play()``
raises the playing flag before it even spawns its task, so by the time the move
has returned the flag is already up. A move with no chime configured simply ends
on the next tick.

This replaced a narrower rule. The screen used to watch the armed alarm slot and
put the dog up for five seconds when it moved, that being its only local sign of
a dispense; it missed every move the console and the phone made, and it fired
wrongly when ``time_set`` re-armed the alarm without the drum turning at all.

The observer is hooked up only after the start-up sequence has finished, since
the drum's own position-finding move would otherwise have ended the splash
early: it plays no chime, so there would have been nothing to wait on.
``dog [seconds]`` still does it by hand from the console, and ``dog 0`` cuts one
short.

He is also the first thing on the glass at start-up. ``screen_start()`` takes a
``splash`` flag that puts him up as the very first frame rather than having him
asked for afterwards, so there is no moment where the clock is drawn and the
backlight then comes up on the wrong thing. He is held rather than timed, with
``SCREEN_CELEBRATE_HOLD``, because what he is covering is the drum going looking
for itself and nobody knows in advance how long that takes; a portrait that
vanished mid-move would read as a fault rather than a greeting. The boot then
runs:

- the dog appears and the backlight fades up over him,
- BLE and the console come up behind him,
- ``dispenser_find_position()`` turns the drum until it knows which slot it is
  on, which is the one part of the boot that takes a visible moment,
- a tada, on success only: that call stays deliberately quiet, being a move
  nobody asked for, and a triumphant noise over a drum that has no idea where it
  is would be a lie,
- two seconds later the face arrives.

``screen_resume()`` ends a celebration as well as lifting a suspend, which
matters more than it sounds: without it, asking for the face back while the dog
is held would set the repaint and then be stepped over every tick until he
finished, and anything drawn over the panel in the meantime would simply stay
there.

Both come out of one master, ``components/screen/assets/dog_source.png``,
through ``tools/img2rgb565.py``::

    ~/.venvs/imgtools/bin/python tools/img2rgb565.py \
        components/screen/assets/dog_source.png \
        --trim --size 67x88 --name dog_badge --out components/screen/assets
    ~/.venvs/imgtools/bin/python tools/img2rgb565.py \
        components/screen/assets/dog_source.png \
        --trim --size 144x188 --name dog_large --out components/screen/assets

The master is kept next to what is generated from it, so those two commands are
the whole story for replacing the artwork. They write a ``.c`` and ``.h`` pair
per asset, a ``const uint16_t`` array in host byte order, which is what
``gc9a01a_draw_bitmap()`` takes; it byte swaps into its own DMA buffer on the
way out, so the asset never has to care about the wire order, and being const it
lives in flash rather than RAM. The dog's two together are 64 kB of an 8 MB
partition, and the Bluetooth icon another 2 kB.

``--trim`` is what cuts the transparent border away, and it is the whole reason
the assets can be cut to his shape: it measures where the drawing actually is
rather than trusting the frame it arrived in. ``--size`` then takes ``WxH`` as
well as a single number, and warns if the two aspects differ by more than a
couple of per cent, since that is the point at which a stretch starts to show.
Both of these come out under that, so neither is distorted.

There is a trap in ``--crop`` that is worth naming, because it caught this
layout once. Trimming a slice off one edge and then padding the result back out
to square gains nothing at all: the scale that follows is set by the longest
side either way, so the subject comes out exactly as small as it started and
short of whatever was cut. It only looks bigger because what was removed is no
longer there to draw the eye. ``--crop`` therefore does not pad, and choosing the
output shape is ``--size``'s job.

Three things the converter has to do that are easy to miss. RGB565 keeps five
bits of red and blue and six of green, so smooth gradients band, worst of all in
the dark tones where a black dog spends most of its time; an ordered dither
scatters the rounding error rather than letting it pool into visible steps. The
panel has no alpha, so transparency is composited onto a background colour,
black by default because that is what the face is drawn on — anything else
would show up as a box around him. And artwork that arrives on white rather than
on alpha needs its background taken off first, since a straight colour key would
punch holes in his cream muzzle and his teeth. Pillow is not in the ESP-IDF
image; the script prints the venv incantation if it is missing.

Reading an asset back out of its ``.c`` file and rendering it is the way to
check all of that at once — the emitter, the dither and the compositing
together, rather than only the intent.

A tick runs twice a second and repaints only the digits whose value actually
moved, which in the usual second is one digit and seven small rectangles. The
countdown comes from the slot the RTC alarm is armed for, so the armed slot
changing is also this side's only sign that a treat has just been dispensed;
that is worth waking a dimmed panel for, since it is the moment somebody might
look. Without an RTC the clock shows dashes, and without a schedule so does the
countdown.

Anything that wants the whole panel to itself has to say so: ``display_test``
and ``display_fill`` pause the face, and ``screen`` paints it again and takes
the panel back.

The layout is also deliberately shaped around the image sticking advice below.
A redraw is *not* reported to the screen care layer as activity, because a
countdown ticking once a second would otherwise reset the idle clock once a
second and the panel would never dim or nudge. Blanking is switched off instead
of that: the board has no button, no touch panel and nothing else a person can
press, so a blanked panel would stay dark until the next feeding time or a
phone connection woke it. Dimming to a third after five idle minutes is enough,
and a black field with thin white strokes is the least stressed state the glass
has. Turning blanking back on in ``main.c`` is a supported choice and the face
copes with it, the display then lighting up at each feeding time and going dark
ten minutes later.

Image sticking
~~~~~~~~~~~~~~

The module datasheet devotes its last section to image sticking, and is blunt
about it: a fixed picture polarizes the liquid crystal, the crystal then cannot
relax back, and a ghost of the old picture stays visible under the new one. It
is inherent to the technology and explicitly not covered by warranty, so the
whole remedy is in how the panel is driven. A dispenser sitting on a shelf
showing the same next feeding time all day is exactly the case the datasheet
describes, so the policy lives in the driver rather than in whoever draws the
screen.

An optional task watches how long the picture has been unchanged and works down
the datasheet's own list of advice:

- after a minute, the whole picture is walked a couple of pixels up and back
  down again, which the controller does through its vertical scroll register
  and therefore costs no repaint. The point is that the boundary between two
  colours does not stand over the same row of crystal for hours.
- after five minutes the backlight fades to 20 percent.
- after ten minutes the panel goes into sleep mode, which blanks it, stops the
  DC/DC converter and the oscillator, and drains the charge off the glass.
  Frame memory survives it, so the picture is still there and still right when
  the panel comes back.

Whoever draws calls ``gc9a01a_care_touch()`` before each update. That resets the
clock, undoes the nudge so drawing coordinates land where they are expected, and
wakes and relights a sleeping panel, so the drawing code never has to know any
of this happened. Where a layout does have to sit still for hours, the datasheet
asks for mid grey and block fills rather than hard lines, and for the colours
either side of a long lived border to sit symmetrically around mid grey;
``GC9A01A_GREY_MID`` is there for that.

Recovery is the other half of the advice: black on a powered panel for four to
six hours, and quicker if the panel is warm. ``display_soak <minutes>`` does
that. Deliberately sleep mode is *not* used for a soak, so that the crystal is
actively driven to its relaxed state rather than just left unpowered.

Serial console
--------------

The firmware starts a console REPL on UART0. Available commands:

- ``speed <0-100>`` — set the motor speed in percent (0 lets the motor coast)
- ``direction <0|1>`` — set the rotation direction (0 = forward, 1 = reverse)
- ``stop`` — stop the motor (coast)
- ``brake`` — actively brake the motor
- ``play <melody> <times> <0-100>`` — play a melody a number of times at the
  given volume (melody 0 = For the Longest Time, melody 1 = tada)
- ``quiet`` — stop melody playback
- ``time_set <YYYY-MM-DD> <HH:MM:SS>`` — set the RTC date and time
- ``time_get`` — show the RTC date and time
- ``i2c_scan`` — probe every 7-bit address on the I2C bus
- ``pd_status`` — show the CH224A protocol status and available current
- ``pd_dump`` — read every documented CH224A register
- ``pd_read <reg>`` — read a single CH224A register
- ``pd_voltage <gear>`` — request a USB-PD voltage gear (0 = 5V, 1 = 9V; higher
  gears are refused because the board cannot take them)
- ``supply`` — show the +9V rail voltage
- ``supply_calibrate <mV>`` — trim the rail voltage gain against a meter reading
- ``supply_range [<under mV> <over mV> [<hyst mV>]]`` — show or set the
  acceptable rail voltage window
- ``status`` — show the current speed, direction and playback state
- ``home`` — turn the drum until it parks on the home magnet
- ``next`` — turn the drum on to the next magnet
- ``dispense_speed [<1-100>]`` — show or set the speed ``home`` and
  ``next`` drive at
- ``schedule`` — show the automatic dispensing times and the one the RTC
  alarm is armed for
- ``ble`` — show the state of the phone link and the pairing window
- ``ble_pair`` — re-open the pairing window so another phone can bond
- ``ble_forget`` — delete every stored bond
- ``display`` — show the panel state and what the screen care policy is doing
- ``display_test`` — draw a test pattern, for checking the wiring and orientation
- ``display_fill <color>`` — fill the panel with a named colour or an RGB565 value
- ``display_backlight [<pct> [<ms>]]`` — show or set the backlight duty cycle,
  optionally ramping to it
- ``display_rotation <0-3>`` — rotate the panel by quarter turns
- ``display_soak <minutes>`` — hold the panel on black to unwind image sticking
- ``screen`` — repaint the clock face and take the panel back from a test pattern
- ``dog [<seconds>]`` — put the dog up on the panel, as a dispensed treat does
- ``help`` — list all commands
