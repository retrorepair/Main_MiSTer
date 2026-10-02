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
#   Logic level: those inputs were driven by the CD DSP, so they take whatever it
#   ran at. Measure the DSP's Vcc -- 3.3 V means the Pico drives them directly, 5 V
#   means a 74HCT buffer between (HCT accepts a 3.3 V input).

import board
import digitalio
import pwmio
import time

# ---------------------------------------------------------------- configuration

# "hbridge" for the BA5977FP, confirmed from its pin table: each PWM channel has
# TWO input pins, FIN and RIN, not one pin with 50% duty as centre. PWM on FIN with
# RIN low drives forward, and the other way round for reverse. "btl" is kept for a
# driver that really does take a single centred input.
MODE        = "hbridge"      # "hbridge": 2 pins/motor (BA5977FP, L293D, DRV8833)
                             # "btl":     1 pin/motor, 50% duty = stop

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

_a = pwmio.PWMOut(SLED_PINS[0], frequency=PWM_HZ, duty_cycle=0)
_b = pwmio.PWMOut(SLED_PINS[1], frequency=PWM_HZ, duty_cycle=0)


def _u16(frac):
    if frac < 0.0:
        frac = 0.0
    elif frac > 1.0:
        frac = 1.0
    return int(frac * 65535)


def drive_inward(duty):
    if MODE == "btl":
        _a.duty_cycle = _u16(0.5 - duty * 0.5)   # below centre = inward
        _b.duty_cycle = 0
    else:
        _a.duty_cycle = 0
        _b.duty_cycle = _u16(duty)


def coast():
    if MODE == "btl":
        _a.duty_cycle = _u16(0.5)                # centre = no drive
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
