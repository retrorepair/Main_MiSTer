# boot.py -- runs once at power-up, before code.py.
#
# 1. Enable a SECOND USB serial port. The first stays the REPL console, which is how
#    the board is debugged; the second ("data") carries the servo protocol that
#    servo_fw.py speaks, so the two never get mixed up.
# 2. Let the running code write the filesystem (it saves the sled polarity in
#    /dir.txt). Measured on the bench: supervisor.runtime.usb_connected reads False
#    here even when the board is plugged into a PC, because USB has not enumerated
#    yet, so this always makes the drive writable by the code and read-only to the PC.
#    To copy files from a PC, either hold the board in the bootloader or write them
#    through the REPL console (see HANDOFF.md).

import storage
import supervisor
import usb_cdc

usb_cdc.enable(console=True, data=True)

if not supervisor.runtime.usb_connected:
    storage.remount("/", readonly=False)
