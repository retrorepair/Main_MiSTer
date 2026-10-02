# sled_cal.py -- characterise a CD sled for acoustic mirroring
#
# MicroPython for RP2040. Copy to the Pico as main.py (or paste at the REPL) and
# call the functions below. MicroPython rather than the C SDK on purpose: this is a
# measuring tool, not the final firmware, and drag-and-drop beats a toolchain.
#
# The question it exists to answer: does the sled run SMOOTHLY at 16-21 mm/s, which
# is a Mega CD's sled velocity, or does static friction stall it first? Everything
# else about the project depends on that answer, so measure it before building more.
#
# WIRING -- read this before powering anything
#
#   The PS1 board stays whole and powered from its own PSU. Cut the traces from the
#   CD DSP to the driver IC's (IC722) input pins, then drive those pads from the
#   Pico. Tie Pico GND to PS1 GND.
#
#   Identify IC722's MUTE / standby / SW pins and drive them to the ENABLED state.
#   Left floating after you cut the DSP's traces, the driver may stay muted, which
#   looks exactly like a wiring fault.
#
#   If IC722's input threshold wants 5 V logic, put a 74HCT buffer between -- HCT
#   specifically, because its TTL thresholds accept a 3.3 V input.
#
# SAFETY
#
#   Every run is time-capped and every path stops the motors in a finally block.
#   Inward runs self-terminate on the limit switch. Outward runs cannot -- there is
#   no outer switch -- so they end on a timeout, and the sled stalls against the
#   outer stop for the remainder. Keep SEAT_MS only as long as it needs to be.

from machine import Pin, PWM
import time

# ---------------------------------------------------------------- configuration

# One pin per motor, or two? A BTL driver of the BA5947FP class takes PWM directly
# with an internal filter, so a single pin per channel carries both direction and
# magnitude: 50% duty is stop, above is one way, below is the other. A plain
# H-bridge (L293D, DRV8833) needs two pins per motor instead. Set to match.
MODE          = "btl"        # "btl" (1 pin/motor, centre = stop) or "hbridge" (2 pins)

SLED_PINS     = (2, 3)       # btl: (pwm, unused). hbridge: (in1, in2)
SPIN_PINS     = (4, 5)
LIMIT_PIN     = 6            # inner limit switch
LIMIT_ACTIVE_LOW = True      # most are: closed to ground when the sled is home

PWM_HZ        = 25000        # above the driver's internal filter corner, and above
                             # hearing, so the PWM itself is not part of the noise

STROKE_MM     = 34.0         # measure yours once with calipers: the distance the
                             # lens travels from the limit switch to the outer stop.
                             # A CD's data area is radius 25-58 mm, so 34 is typical
                             # but mechanisms differ and the number matters.

SEAT_MS       = 3000         # outward push to seat against the outer stop
SEAT_DUTY     = 0.45         # gentle: it stalls there for the rest of the time
MAX_RUN_MS    = 6000         # hard cap on any single motor run

TARGET_LO, TARGET_HI = 16.0, 21.0    # a Mega CD's sled, mm/s

# ---------------------------------------------------------------------- hardware

_sled = [PWM(Pin(p)) for p in SLED_PINS]
_spin = [PWM(Pin(p)) for p in SPIN_PINS]
for _p in _sled + _spin:
    _p.freq(PWM_HZ)

_limit = Pin(LIMIT_PIN, Pin.IN, Pin.PULL_UP)


def _u16(frac):
    frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
    return int(frac * 65535)


def _drive(pwms, duty, outward):
    """duty is 0..1 of full effort; outward is the direction flag."""
    if MODE == "btl":
        # Centre is stop; deviation either side is direction and magnitude.
        level = 0.5 + (duty * 0.5 if outward else -duty * 0.5)
        pwms[0].duty_u16(_u16(level))
        pwms[1].duty_u16(0)
    else:
        a, b = (duty, 0.0) if outward else (0.0, duty)
        pwms[0].duty_u16(_u16(a))
        pwms[1].duty_u16(_u16(b))


def _coast(pwms):
    if MODE == "btl":
        pwms[0].duty_u16(_u16(0.5))      # centre = no drive
        pwms[1].duty_u16(0)
    else:
        pwms[0].duty_u16(0)
        pwms[1].duty_u16(0)


def stop():
    """Everything off. Safe to call at any time."""
    _coast(_sled)
    _coast(_spin)


def sw():
    """Is the sled at the inner limit? Use this to check switch polarity."""
    v = _limit.value()
    return (v == 0) if LIMIT_ACTIVE_LOW else (v == 1)


# ------------------------------------------------------------------ measurement

def home(duty=0.45, timeout_ms=MAX_RUN_MS):
    """Drive inward until the limit switch asserts. Returns ms taken, or None."""
    if sw():
        return 0
    t0 = time.ticks_ms()
    try:
        _drive(_sled, duty, outward=False)
        while not sw():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                print("home: TIMEOUT -- switch never asserted."
                      " Check LIMIT_ACTIVE_LOW, the wiring, and the direction sign.")
                return None
            time.sleep_ms(1)
        return time.ticks_diff(time.ticks_ms(), t0)
    finally:
        stop()


def seat(duty=SEAT_DUTY, ms=SEAT_MS):
    """Push outward to rest against the outer stop, so position is known."""
    try:
        _drive(_sled, duty, outward=True)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


def stroke_time(duty, settle_ms=250):
    """Seat at the outer stop, then time a full inward run to the limit switch.

    Inward, because that is the only direction that can tell us it has arrived.
    Returns (ms, mm_per_s) or (None, None).
    """
    seat()
    time.sleep_ms(settle_ms)
    if sw():
        print("stroke_time: already at the limit after seating -- SEAT_MS too short,"
              " or the direction sign is inverted.")
        return (None, None)
    ms = home(duty)
    if ms is None:
        return (None, None)
    return (ms, STROKE_MM * 1000.0 / ms)


def pulse(duty=0.5, ms=500, outward=True):
    """Drive briefly, for poking at it and measuring with a ruler."""
    try:
        _drive(_sled, duty, outward)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


def spin(duty=0.4, ms=2000):
    """Run the spindle, to hear it and confirm the pair is identified correctly."""
    try:
        _drive(_spin, duty, outward=True)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


def cal(duties=(0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.75, 1.00)):
    """The money command: a duty -> velocity table, and where the target sits.

    Run with nothing else touching the mechanism. Each row seats the sled against
    the outer stop and times a full inward run, so the distance is STROKE_MM every
    time and only the duty varies.
    """
    print("stroke %.1f mm, target %.0f-%.0f mm/s (a Mega CD)\n"
          % (STROKE_MM, TARGET_LO, TARGET_HI))
    print("  duty    time     velocity   verdict")
    print("  ----  -------  ----------   -------")
    rows = []
    for d in duties:
        ms, v = stroke_time(d)
        if ms is None:
            print("  %4.2f      ----        ----   stalled or no switch" % d)
            continue
        if v < TARGET_LO * 0.6:
            verdict = "too slow"
        elif v < TARGET_LO:
            verdict = "slow side"
        elif v <= TARGET_HI:
            verdict = "*** IN RANGE ***"
        elif v <= TARGET_HI * 1.6:
            verdict = "fast side"
        else:
            verdict = "too fast"
        print("  %4.2f  %5d ms  %6.1f mm/s   %s" % (d, ms, v, verdict))
        rows.append((d, v))
        time.sleep_ms(400)

    print()
    if not rows:
        print("Nothing moved. Work through: sw() should flip when you nudge the sled")
        print("by hand; pulse() should move it; and IC722's mute/standby pins must be")
        print("driven to enabled now that the DSP's traces are cut.")
        return rows

    inr = [r for r in rows if TARGET_LO <= r[1] <= TARGET_HI]
    slowest = min(rows, key=lambda r: r[1])
    if inr:
        print("Mega CD velocity is reachable at duty %s." %
              ", ".join("%.2f" % d for d, _ in inr))
        print("That is the result worth having: a seek becomes ONE continuous sweep")
        print("at the right speed, and all the segmentation the USB path needs goes away.")
    else:
        print("Nothing landed in 16-21 mm/s. Slowest usable was %.1f mm/s at duty %.2f."
              % (slowest[1], slowest[0]))
        if slowest[1] > TARGET_HI:
            print("Static friction is setting the floor. Worth trying before giving up:")
            print("  - a little grease on the rails, or working it in by hand first")
            print("  - lower PWM_HZ (try 2000): the cogging helps break stiction")
            print("  - a brief kick at high duty, then drop to the low duty")
    return rows


def check():
    """Run first. Confirms the switch and both motors before anything is timed."""
    print("limit switch now: %s" % ("AT HOME" if sw() else "not at home"))
    print("move the sled by hand and call sw() again -- it must change")
    print()
    print("nudging the sled outward...")
    pulse(0.5, 400, outward=True)
    print("  switch now: %s" % ("AT HOME" if sw() else "not at home"))
    print("nudging the sled inward...")
    pulse(0.5, 400, outward=False)
    print("  switch now: %s" % ("AT HOME" if sw() else "not at home"))
    print()
    print("spindle for 2 s -- you should hear it")
    spin(0.4, 2000)
    print()
    print("if the sled moved the wrong way, swap SLED_PINS or the motor leads.")
    print("then: cal()")


stop()
print("sled_cal ready. check() first, then cal(). stop() at any time.")
