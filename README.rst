Treat Dispenser
===============

Firmware for the treat dispenser board (ESP32-S3-WROOM-1-N16). The board
drives a DC motor through a TI DRV8871 H-bridge (IN1 on GPIO4, IN2 on GPIO5)
and a speaker through a MAX98357A I2S amplifier (BCLK on GPIO16, LRCLK on
GPIO17, DIN on GPIO18, SD_MODE on GPIO21). An RV-3028-C7 RTC (0x52) and the
CH224A USB-PD sink controller (0x22 or 0x23) share an I2C bus on SDA GPIO6 and
SCL GPIO7. A 100k/27k divider (R5/R8) brings the +9V rail down to GPIO2 so the
firmware can measure it. Developed with ESP-IDF v5.5.1.

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

Serial console
--------------

The firmware starts a console REPL on UART0. Available commands:

- ``speed <0-100>`` — set the motor speed in percent (0 lets the motor coast)
- ``direction <0|1>`` — set the rotation direction (0 = forward, 1 = reverse)
- ``stop`` — stop the motor (coast)
- ``brake`` — actively brake the motor
- ``play <melody> <times> <0-100>`` — play a melody a number of times at the
  given volume (melody 0 = For the Longest Time)
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
- ``help`` — list all commands
