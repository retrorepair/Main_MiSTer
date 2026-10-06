# code.py -- CD sled velocity test. One run per power-up, no terminal needed.
#
# CircuitPython on an RP2040. Flash CircuitPython (drag the .uf2 onto the RPI-RP2
# drive), then copy boot.py and this file onto the CIRCUITPY drive. That is the whole
# setup: no toolchain, no terminal, nothing to install.
#
#
# WHAT IT DOES, each time you apply power
#
#   1. works out which duty is next, from what is already in results.csv
#   2. waits 3 s, LED blinking, so you can get your hands clear
#   3. drives the sled INWARD at that duty, timing until the limit switch closes
#   4. appends the result and signals it on the LED
#
#   So the sequence is: wind the sled to the OUTER stop, apply power, wait for the
#   confirmation blinks, remove power. Repeat. After eight power-ups the table is
#   complete, and plugging the Pico into a PC shows results.csv on the drive.
#
#   The sled MUST start at the hard outer stop every time. That is what makes the
#   distance identical across runs, which is the whole basis of the measurement.
#
#
# POWER IT FROM THE PS1 (or a phone charger) WHILE TESTING, not from the PC.
#   See boot.py: with a USB data host attached the filesystem belongs to the host
#   and the result cannot be written. You get a continuous fast blink if so.
#
#
# LED CODES
#   3 slow blinks, then solid during the run
#   N blinks after        -- run recorded, N = how many runs are now stored
#   solid on              -- all duties done, nothing left to do
#   5 fast, repeating     -- sled was already at the inner switch: wind it out first
#   long-short, repeating -- never reached the switch: stalled, or the duty is too low
#   continuous fast blink -- filesystem is read-only: you are plugged into a PC
#
#
# WIRING
#   The PS1 board stays whole on its own PSU. Cut the traces from the CD DSP to the
#   driver IC's input pins and drive those pads from the Pico. Tie Pico GND to PS1
#   GND. The 8 V supplies (pins 10, 19, 28) stay on the board: the Pico only ever
#   drives logic-level inputs, and the IC swings 8 V to the motors.
#
#   HOLD PIN 20 (MUTE) AT ITS UN-MUTED LEVEL. It was driven by the CD DSP, so once
#   that trace is cut it floats and the driver may stay muted -- which looks exactly
#   like a wiring fault. Measure pin 20 on the running PS1 first and hold it there.
#
#   Logic level: NO BUFFER NEEDED. The PU-22 board's digital rail is DIG 3.5 V and
#   the CD DSP (IC732) runs from it, so the inputs it was driving on IC722 take
#   3.5 V logic -- a Pico's 3.3 V output drives them directly. From the service
#   manual block diagram, which also labels MOT +8V as a dedicated motor rail into
#   IC722, so the 8 V arrives on its own.
#
#   The traces to cut are between IC732 (CD DSP) and IC722 (DRIVER). The optical
#   block is a KSM-440AEM; of its two connectors, the one going to IC723 CD-RF is
#   the laser flex and can be ignored entirely -- the other is the motor connector.
#
#
# WIRING FOR ch4, WHICH IS WHERE THE SLED TURNED OUT TO BE
#
#   The sled motor is on ch4OUTR/ch4OUTF (pins 15, 16). ch4 is the analogue channel,
#   so it takes a VOLTAGE on pin 24, not a PWM pair. Turn the Pico's PWM into one
#   with a low-pass filter:
#
#       GP2 ----[ 1k ]----+---- IC722 pin 24  (ch4IN)
#                         |
#                      [ 10uF ]
#                         |
#                        GND
#
#   1k and 10uF give a 10 ms time constant: ripple at 25 kHz is nothing, and 10 ms
#   is far quicker than any gesture. Keep the series resistor low -- pin 24 sits
#   behind resistors around 100k, so a 10k source would drop a noticeable fraction
#   of the signal across itself. Exact values hardly matter otherwise, because duty
#   is calibrated against measured velocity anyway.
#
#   Pin 26 (OUTVref) is the reference the output is compared against, and pin 25
#   (ch4CAPA) is its external capacitor. BOTH ARE ALREADY FITTED on the board --
#   leave them. Just measure pin 26 and set ANALOG_CENTRE = that voltage / 3.3.
#
#   Lift pins 24, 20 and 3 to disconnect the DSP, then drive all three from the Pico.
#   Leave pins 10, 19, 28 (the 8 V rails), pins 25 and 26, and the motor connector
#   exactly as they are.

import board
import digitalio
import pwmio
import time

# ---------------------------------------------------------------- configuration

# "analog" because the sled is on the BA5977FP's ch4, which is the odd channel out:
# ONE input (pin 24, ch4IN) whose voltage is compared against an external reference
# (pin 26, OUTVref). Above the reference drives one way, below drives the other, and
# at the reference it stops. Channels 1-3 would have been a pair of PWM pins
# ("hbridge"), which is easier -- ch4 needs the Pico's PWM turned into a DC level by
# an RC filter first. See WIRING below.
MODE        = "analog"       # "analog":  1 pin via RC filter, centred on ANALOG_CENTRE
                             # "hbridge": 2 pins/motor (ch1-3, L293D, DRV8833)

# Duty that lands the filtered output ON the reference, i.e. motor stopped. With a
# 3.3 V PWM this is roughly OUTVref / 3.3 -- so measure pin 26 and divide. A reference
# at 1.65 V gives 0.50; at 1.75 V gives 0.53. Getting this close matters: if the
# "stopped" duty is off, the sled creeps whenever it should be still.
ANALOG_CENTRE = 0.50

# If the sled runs the wrong way, flip this rather than rewiring.
INVERT_DIRECTION = False

# For the BA5977FP these are (FIN, RIN) of whichever channel drives the sled.
#
#   BA5977FP, from the datasheet pin table
#   ---------------------------------------------------------------
#    1,2,27  OPIN-, OPIN+, OPOUT   spare op-amp
#    3       SW                    ch4 input select (NOT a master enable)
#    4,5     ch1FIN, ch1RIN        ch1 PWM in      ->  out 14, 13
#    6,7     ch2FIN, ch2RIN        ch2 PWM in      ->  out 12, 11
#    8,21    GND
#    9       VrefIN                internal Vref
#    10      PowVcc (ch1,2)        8 V, leave on the board
#    11,12   ch2OUTR, ch2OUTF      ch2 motor
#    13,14   ch1OUTR, ch1OUTF      ch1 motor
#    15,16   ch4OUTR, ch4OUTF      ch4 motor
#    17,18   ch3OUTF, ch3OUTR      ch3 motor
#    19      PowVcc (ch3,4)        8 V, leave on the board
#    20      MUTE                  hold at its un-muted level -- MEASURE IT on the
#                                  running PS1 before cutting anything
#    22,23   ch3RIN, ch3FIN        ch3 PWM in      ->  out 18, 17
#    24,25,26 ch4IN, ch4CAPA, OUTVref   ch4 is ANALOGUE, not PWM
#    28      PreVcc                8 V, leave on the board
#
# Buzz the motor connector to pins 11-18 to find which channel is the sled. If it
# lands on ch1/ch2/ch3 it is a PWM pair and this code drives it directly. If it
# lands on ch4 that channel wants a voltage, so a PWM-plus-RC filter is needed.
SLED_PINS   = (board.GP2, board.GP3)    # hbridge: (FIN, RIN). btl: (pwm, unused)
LIMIT_PIN   = board.GP6
LIMIT_ACTIVE_LOW = True      # closed to ground when the sled is home

# Pin 20 of the BA5977FP. Driven by the CD DSP originally, so once that trace is
# lifted it floats and the driver may sit muted -- which looks exactly like a wiring
# fault. Let the Pico hold it instead of hard-wiring it: if the polarity turns out to
# be the other way round, change the flag rather than the solder.
#
# Measure pin 20 on the running PS1 first and set MUTE_UNMUTED_HIGH to match what you
# see while the drive is working normally. Set MUTE_PIN to None if you would rather
# tie it to a rail by hand.
MUTE_PIN          = board.GP7
MUTE_UNMUTED_HIGH = True

# Pin 3, SW: the BA5977FP's ch4 input switch, documented as H -> ON, L -> OFF. It did
# not matter while the sled looked like it was on a PWM channel; it matters now,
# because it gates the very channel being driven. Measure pin 3 on the running PS1
# and match what you see.
SW_PIN        = board.GP8
SW_ON_HIGH    = True

PWM_HZ      = 25000          # above the driver's internal filter, and above hearing

# Tried in order, one per power-up. Coarse on purpose: this is a first look at
# where the usable range is, not a fine sweep.
DUTIES      = (0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.75, 1.00)

SETTLE_S    = 3.0            # pause before moving, so you can get clear
MAX_RUN_S   = 8.0            # hard cap on the motor run
RESULTS     = "/results.csv"

# ---------------------------------------------------------------------- hardware

_led = digitalio.DigitalInOut(board.LED)
_led.direction = digitalio.Direction.OUTPUT

_limit = digitalio.DigitalInOut(LIMIT_PIN)
_limit.direction = digitalio.Direction.INPUT
_limit.pull = digitalio.Pull.UP

# Hold the driver un-muted for as long as this runs. Done before anything else, so
# the very first motion attempt is not fighting a muted output stage.
if MUTE_PIN is not None:
    _mute = digitalio.DigitalInOut(MUTE_PIN)
    _mute.direction = digitalio.Direction.OUTPUT
    _mute.value = MUTE_UNMUTED_HIGH

# ch4's input switch, held on for as long as this runs.
if SW_PIN is not None:
    _sw = digitalio.DigitalInOut(SW_PIN)
    _sw.direction = digitalio.Direction.OUTPUT
    _sw.value = SW_ON_HIGH

_a = pwmio.PWMOut(SLED_PINS[0], frequency=PWM_HZ, duty_cycle=0)
_b = pwmio.PWMOut(SLED_PINS[1], frequency=PWM_HZ, duty_cycle=0)


def _u16(frac):
    if frac < 0.0:
        frac = 0.0
    elif frac > 1.0:
        frac = 1.0
    return int(frac * 65535)


def drive_inward(duty):
    if MODE == "analog":
        # Swing away from the reference. How far below (or above) sets the speed,
        # and the headroom is whichever side of ANALOG_CENTRE is smaller.
        span = min(ANALOG_CENTRE, 1.0 - ANALOG_CENTRE)
        off  = duty * span
        _a.duty_cycle = _u16(ANALOG_CENTRE + off if INVERT_DIRECTION
                             else ANALOG_CENTRE - off)
        _b.duty_cycle = 0
    else:
        a, b = (duty, 0.0) if INVERT_DIRECTION else (0.0, duty)
        _a.duty_cycle = _u16(a)
        _b.duty_cycle = _u16(b)


def coast():
    if MODE == "analog":
        _a.duty_cycle = _u16(ANALOG_CENTRE)      # on the reference = stopped
        _b.duty_cycle = 0
    else:
        _a.duty_cycle = 0
        _b.duty_cycle = 0


def at_home():
    return (not _limit.value()) if LIMIT_ACTIVE_LOW else _limit.value()


# --------------------------------------------------------------------- led codes

def blink(n, on=0.12, off=0.18):
    for _ in range(n):
        _led.value = True
        time.sleep(on)
        _led.value = False
        time.sleep(off)


def forever(pattern):
    """Repeat a (on_s, off_s) pattern until power is removed."""
    while True:
        for on, off in pattern:
            _led.value = True
            time.sleep(on)
            _led.value = False
            time.sleep(off)
        time.sleep(0.6)


# ------------------------------------------------------------------ persistence

def read_results():
    rows = []
    try:
        with open(RESULTS) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("duty"):
                    continue
                parts = line.split(",")
                if len(parts) >= 2:
                    try:
                        rows.append((float(parts[0]), int(parts[1])))
                    except ValueError:
                        pass
    except OSError:
        pass
    return rows


def append_result(duty, ms):
    new = False
    try:
        with open(RESULTS) as f:
            f.read(1)
    except OSError:
        new = True
    with open(RESULTS, "a") as f:
        if new:
            f.write("duty,ms,note\n")
        f.write("%.2f,%d,%s\n" % (duty, ms, "ok" if ms > 0 else "stalled"))


# ------------------------------------------------------------------------- main

def main():
    coast()
    done = read_results()

    # Writable? If not we are plugged into a PC, and this run cannot be recorded.
    try:
        with open(RESULTS, "a") as f:
            pass
    except OSError:
        forever([(0.06, 0.06)])          # continuous fast blink

    if len(done) >= len(DUTIES):
        _led.value = True                # solid: nothing left to do
        while True:
            time.sleep(1)

    duty = DUTIES[len(done)]

    if at_home():
        # Nothing to time from here.
        forever([(0.07, 0.07)] * 5)      # 5 fast, repeating

    blink(3, 0.25, 0.25)
    time.sleep(SETTLE_S)

    ms = 0
    _led.value = True
    start = time.monotonic()
    try:
        drive_inward(duty)
        while not at_home():
            if time.monotonic() - start > MAX_RUN_S:
                break
            time.sleep(0.001)
        else:
            ms = int((time.monotonic() - start) * 1000)
    finally:
        coast()
        _led.value = False

    append_result(duty, ms)

    if ms <= 0:
        forever([(0.6, 0.15), (0.12, 0.6)])     # long-short: stalled
    blink(len(done) + 1, 0.18, 0.22)            # how many runs are stored now
    while True:
        blink(1, 0.05, 1.95)                    # quiet heartbeat: safe to power off


main()
