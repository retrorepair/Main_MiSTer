# diag.py -- find out WHICH link is broken, with a multimeter.
#
# Copy this to the CIRCUITPY drive as code.py (rename your existing code.py to
# code_cal.py first so you can put it back). It never writes a file, so boot.py and
# the USB/filesystem question do not matter: run it with the Pico on the PC or on a
# charger, whichever is easier.
#
# It loops forever through five stages, each held long enough to measure. The LED
# blinks the stage number at the start of each stage, then:
#
#   stage 1  LIMIT SWITCH   LED mirrors the switch. On = closed (at home).
#   stage 2  CENTRE         drive is un-muted, input held on the reference. STOPPED.
#   stage 3  INWARD         input pulled below the reference.
#   stage 4  OUTWARD        input pushed above the reference.
#   stage 5  MUTED          same as inward, but MUTE asserted. Should be STOPPED.
#
# WHAT TO MEASURE (PS1 powered on; red probe on the pin, black on PS1 ground)
#
#   before anything:  pins 28, 10, 19 of IC722 should read about 8 V.
#                     If they do not, nothing downstream can work.
#
#   stage 2:  pin 24 should read the SAME as pin 26 (OUTVref).
#             motor connector voltage should be about 0 V.
#   stage 3:  pin 24 a little BELOW pin 26.   motor voltage a few volts, one polarity.
#   stage 4:  pin 24 a little ABOVE pin 26.   motor voltage a few volts, OPPOSITE.
#   stage 5:  motor voltage back to about 0 V.
#
# Measure the motor between pins 15 and 16 of IC722 (the sled outputs), or at the
# motor connector.
#
#   motor volts follow the stages, sled does not move  -> the drive is working and the
#       problem is mechanical: stiction, or too little voltage. Raise SWING.
#   pin 24 follows the stages, motor volts never change -> MUTE or SW is the wrong
#       polarity, or the IC is not powered. Flip MUTE_UNMUTED_HIGH / SW_ON_HIGH.
#   pin 24 does NOT follow the stages -> the wire is not reaching the IC leg, or the
#       lifted pin is not actually the one you soldered to.
#   stage 5 does not stop the motor -> pin 20 is not connected to the IC.
#   LED does nothing at all -> the code is not running. Check the file is named
#       code.py, is on the CIRCUITPY drive, and the Pico has CircuitPython on it.

import board
import digitalio
import pwmio
import time

# ------------------------------------------------ keep these the same as code.py
SLED_PIN          = board.GP2        # to IC722 pin 24, through the RC if you fitted it
LIMIT_PIN         = board.GP6
LIMIT_ACTIVE_LOW  = True
MUTE_PIN          = board.GP7        # IC722 pin 20
MUTE_UNMUTED_HIGH = True
SW_PIN            = board.GP8        # IC722 pin 3
SW_ON_HIGH        = True
ANALOG_CENTRE     = 0.50             # pin 26 voltage / 3.3
PWM_HZ            = 25000

# How far from the centre stages 3 and 4 push. 0.3 is gentle. If you see volts at the
# motor but the sled will not move, raise it (0.6, then 1.0).
SWING             = 0.30
STAGE_S           = 8

# --------------------------------------------------------------------------------
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

limit = digitalio.DigitalInOut(LIMIT_PIN)
limit.direction = digitalio.Direction.INPUT
limit.pull = digitalio.Pull.UP

mute = digitalio.DigitalInOut(MUTE_PIN)
mute.direction = digitalio.Direction.OUTPUT

sw = digitalio.DigitalInOut(SW_PIN)
sw.direction = digitalio.Direction.OUTPUT
sw.value = SW_ON_HIGH

pwm = pwmio.PWMOut(SLED_PIN, frequency=PWM_HZ, duty_cycle=0)


def u16(f):
    return int(max(0.0, min(1.0, f)) * 65535)


def unmuted(on):
    mute.value = MUTE_UNMUTED_HIGH if on else (not MUTE_UNMUTED_HIGH)


def drive(offset):
    """offset is -1..+1 of the available swing around the centre."""
    span = min(ANALOG_CENTRE, 1.0 - ANALOG_CENTRE)
    pwm.duty_cycle = u16(ANALOG_CENTRE + offset * span)


def at_home():
    return (not limit.value) if LIMIT_ACTIVE_LOW else limit.value


def label(n):
    led.value = False
    time.sleep(0.4)
    for _ in range(n):
        led.value = True
        time.sleep(0.2)
        led.value = False
        time.sleep(0.25)
    time.sleep(0.3)


def hold(seconds):
    led.value = True
    time.sleep(seconds)
    led.value = False


while True:
    # 1: limit switch. Press it, or slide the sled onto it, and the LED follows.
    unmuted(False)
    drive(0.0)
    label(1)
    end = time.monotonic() + STAGE_S + 4
    while time.monotonic() < end:
        led.value = at_home()
        time.sleep(0.02)

    # 2: centre. Un-muted, input on the reference: the sled must be STOPPED.
    unmuted(True)
    drive(0.0)
    label(2)
    hold(STAGE_S)

    # 3: inward.
    unmuted(True)
    drive(-SWING)
    label(3)
    hold(STAGE_S)

    # 4: outward.
    unmuted(True)
    drive(+SWING)
    label(4)
    hold(STAGE_S)

    # 5: same as 3 but muted. The motor must stop.
    unmuted(False)
    drive(-SWING)
    label(5)
    hold(STAGE_S)
