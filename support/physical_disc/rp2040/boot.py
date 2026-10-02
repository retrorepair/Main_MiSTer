# boot.py -- decide who owns the filesystem, based on how the Pico is powered.
#
# CircuitPython exposes CIRCUITPY as a USB drive, but the host and the running code
# cannot both write to it. So:
#
#   powered WITHOUT a USB data host (from the PS1's 5 V, or a phone charger)
#       -> the filesystem is writable by code, so a test run can log its result
#
#   plugged into a PC
#       -> the filesystem belongs to the host, so the results file can be read off
#
# Which is exactly the intended workflow: run the tests on PS1 power, then plug into
# the PC to collect them.
#
# CONSEQUENCE, and it is the one thing that will catch you out: do NOT run the tests
# with the Pico plugged into the PC. It cannot write the result and code.py will say
# so with a continuous fast blink.

import storage
import supervisor

if not supervisor.runtime.usb_connected:
    # Standalone: let the test write its result.
    storage.remount("/", readonly=False)
# Plugged into a host: leave it alone, so the drive is readable as normal.
