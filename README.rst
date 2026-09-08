Treat Dispenser
===============

Firmware for the treat dispenser board (ESP32-S3-WROOM-1-N16). The board
drives a DC motor through a TI DRV8871 H-bridge (IN1 on GPIO4, IN2 on GPIO5)
and a speaker through a MAX98357A I2S amplifier (BCLK on GPIO16, LRCLK on
GPIO17, DIN on GPIO18, SD_MODE on GPIO21). An RV-3028-C7 RTC (0x52) and the
CH224A USB-PD sink controller (0x22 or 0x23) share an I2C bus on SDA GPIO6 and
SCL GPIO7; the RTC drives its open drain INT pin into GPIO15 against a 4k7
pull-up. A 100k/27k divider (R5/R8) brings the +9V rail down to GPIO2 so the
firmware can measure it.
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
- ``help`` — list all commands
