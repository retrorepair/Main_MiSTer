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
#   2 long, repeating     -- FIRST POWER-UP ONLY: direction found and saved. The sled is
#                            now on the switch: wind it back out and power up again.
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
# WIRING (BA5977FP, IC722). The channel map is from Sony's own PSone service manual
# schematic: it follows each IC output to the 4-pin motor plug CN701.
#
#     SLED    = ch3  outputs pins 17/18, INPUTS pins 22 (RIN) and 23 (FIN)  <- this code
#     SPINDLE = ch4  outputs pins 15/16, input pin 24 (analogue)
#     ch1/ch2 = focus / tracking coils, via the laser flex. Leave pins 4-7 alone.
#
#   Pico  GP4  ->  IC722 pin 23   sled forward input
#   Pico  GP5  ->  IC722 pin 22   sled reverse input
#   Pico  GP7  ->  IC722 pin 20   MUTE: high = running (the schematic shows 3.3 V)
#   Pico  GP8  ->  IC722 pin 3    SW: held LOW (the schematic shows 0 V)
#   Pico  GP2  ->  1k -> pin 24 (+ 10uF to GND)   spindle, parked on its reference
#   Pico  GP6  ->  limit switch, other side to GND
#   Pico  GND  ->  PS1 GND
#
#   Lift pins 22, 23, 20, 3 and 24, and solder to the IC's LEG, not to the pad: the pad
#   still leads to the CD DSP's trace and the signal never reaches the chip.
#   Pin 20 sits between GND (21) and the 7.4 V rail (19) -- check for a bridge.
#   Do NOT connect pins 17/18 (outputs), 4-7, or 10/19/28 (the 8 V rails).
#
#   FIRST POWER-UP finds the sled's direction itself and saves it to /dir.txt, so there
#   is no polarity to get right in advance. See main().
#
#   Normal operating voltages from the same schematic, for comparing a meter against:
#   MUTE(20) 3.3 V, SW(3) 0 V, PowVcc/PreVcc(10,19,28) 7.4 V, VrefIN(9) 3.3 V,
#   ch4IN(24) 1.9 V, OUTVref(26) 1.7 V, ch3 inputs(22,23) about 0 V.

import board
import digitalio
import pwmio
import time

# ---------------------------------------------------------------- configuration

# "hbridge": the sled is on ch3, which is a plain FIN/RIN pair of PWM inputs. PWM on
# FIN with RIN low drives one way, the other way round drives the other, and both low
# is stopped. No filter, reference or SW pin needed. (An earlier version of this file
# drove ch4 in "analog" mode because I took "ch4 is the sled" on trust. The schematic
# says ch4 is the SPINDLE. "analog" is kept for reference but is not used.)
MODE        = "hbridge"      # "hbridge": 2 pins, FIN/RIN. "analog": 1 pin + RC filter

# Only used in "analog" mode. 1.7 V (Sony's OUTVref) / 3.3 V.
ANALOG_CENTRE = 0.515

# Which way is "inward" is found by the first power-up and stored in /dir.txt, then
# loaded into this. False/True are only the starting point.
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
SLED_PINS   = (board.GP4, board.GP5)    # (FIN, RIN) = IC722 pins 23, 22  (ch3)
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

# Pin 3, SW: ch4's input switch only (the spindle), so it does not gate the sled. Sony's
# schematic shows 0 V on it in normal operation, so it is held LOW, which is the state
# the PS1 itself runs in. (Set SW_PIN to None to leave it to a hard wire.)
SW_PIN        = board.GP8
SW_ON_HIGH    = False

# Spindle: ch4, pin 24 through the RC, parked ON its reference so it stays still while
# the sled is being measured. Sony's schematic shows OUTVref at 1.7 V: 1.7 / 3.3.
SPINDLE_PIN       = board.GP2
SPINDLE_STOP_DUTY = 0.515

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


# Hold the spindle still for as long as this runs. Without a PWM output on GP2 the
# RC would float and ch4 could wander off its reference.
_spindle = pwmio.PWMOut(SPINDLE_PIN, frequency=PWM_HZ, duty_cycle=_u16(SPINDLE_STOP_DUTY))


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
    # .value is a PROPERTY in CircuitPython, not a method as it is in MicroPython.
    # This used to read _limit.value() and raised "'bool' object is not callable" the
    # first time it ran, which killed code.py silently right after it created an empty
    # results.csv. No test was ever run. Carried over from the MicroPython version.
    return (not _limit.value) if LIMIT_ACTIVE_LOW else _limit.value


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
            if not f.read(1):
                new = True           # the file exists but is empty
    except OSError:
        new = True
    with open(RESULTS, "a") as f:
        if new:
            f.write("duty,ms,note\n")
        f.write("%.2f,%d,%s\n" % (duty, ms, "ok" if ms > 0 else "stalled"))


# ------------------------------------------------------------------ direction

DIRFILE = "/dir.txt"


def load_direction():
    """True/False once the first power-up has found it, None before that."""
    try:
        with open(DIRFILE) as f:
            return f.read().strip() == "1"
    except OSError:
        return None


def save_direction(inverted):
    with open(DIRFILE, "w") as f:
        f.write("1" if inverted else "0")


def try_direction(inverted, seconds, duty=0.45):
    """Drive with this polarity and report whether the limit switch closed.

    The sled starts at the OUTER end, so only genuine inward travel can reach the
    switch. That makes the switch an unambiguous test of which polarity is inward.
    """
    global INVERT_DIRECTION
    INVERT_DIRECTION = inverted
    start = time.monotonic()
    try:
        drive_inward(duty)
        while time.monotonic() - start < seconds:
            if at_home():
                return True
            time.sleep(0.001)
        return False
    finally:
        coast()


# ------------------------------------------------------------------------- main

def main():
    global INVERT_DIRECTION
    coast()

    # Writable? If not we are plugged into a PC, and this run cannot be recorded.
    try:
        with open(RESULTS, "a") as f:
            pass
    except OSError:
        forever([(0.06, 0.06)])          # continuous fast blink

    inverted = load_direction()
    if inverted is None:
        # FIRST POWER-UP EVER: find which polarity is inward, so nothing has to be
        # guessed or swapped. Start with the sled wound to the OUTER stop.
        if at_home():
            forever([(0.07, 0.07)] * 5)  # 5 fast: the sled is already on the switch
        blink(3, 0.25, 0.25)
        time.sleep(SETTLE_S)
        _led.value = True
        found = None
        if try_direction(False, 7.0):
            found = False
        else:
            time.sleep(0.5)
            if try_direction(True, 12.0):
                found = True
        _led.value = False
        if found is None:
            # Nothing reached the switch either way: not driven, or switch unwired.
            forever([(0.6, 0.15), (0.12, 0.6)])     # long-short
        save_direction(found)
        # Two long blinks, repeating: direction saved. The sled is now ON the switch,
        # so wind it back out to the outer stop and power up again to start the table.
        forever([(0.5, 0.3), (0.5, 1.0)])
    INVERT_DIRECTION = inverted

    done = read_results()

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
