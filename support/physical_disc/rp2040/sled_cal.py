# sled_cal.py -- characterise a CD sled for acoustic mirroring
#
# MicroPython for RP2040. Copy to the Pico as main.py (or paste at the REPL) and
# call the functions below. MicroPython rather than the C SDK on purpose: this is a
# measuring tool, not the final firmware, and drag-and-drop beats a toolchain.
#
# The question it exists to answer: does the sled run SMOOTHLY at 16-21 mm/s, which
# is a Mega CD's sled velocity, or does static friction stall it first? Everything
# else depends on that answer, so measure it before building any more.
#
#
# HOW IT MEASURES, given only ONE switch
#
#   The inner limit switch is the only position feedback. So only INWARD runs can
#   tell us they have arrived -- an outward run has nothing to stop it, and driving
#   outward "until it gets there" just stalls against the end stop.
#
#   So nothing here ever drives to the outer end. Instead:
#
#     1. home                       -- inward until the switch asserts. Known origin.
#     2. a REFERENCE pulse outward  -- fixed duty, fixed short time. The sled ends up
#                                      somewhere mid-rail. We do not know where in mm,
#                                      but it is the SAME place every single time.
#     3. time an inward run at the  -- ends on the switch, so it self-terminates.
#        duty under test
#
#   Step 2 is identical for every measurement, so the distance is a constant and the
#   inward time is purely a function of the test duty. One ruler measurement turns
#   that into mm/s: see ref_measure() below, which you run ONCE.
#
#   Nothing stalls except momentarily at the inner switch, which is the condition
#   every CD player homes into by design.
#
#
# WIRING -- read before powering anything
#
#   The PS1 board stays whole, powered from its own PSU. Cut the traces from the CD
#   DSP to the driver IC's (IC722) input pins, then drive those pads from the Pico.
#   Tie Pico GND to PS1 GND.
#
#   Identify IC722's MUTE / standby / SW pins and drive them to the ENABLED state.
#   Left floating once the DSP's traces are cut, the driver may stay muted -- which
#   presents exactly like a wiring fault and is the likeliest reason for "nothing
#   moves".
#
#   If IC722's input threshold wants 5 V logic, put a 74HCT buffer between. HCT
#   specifically: its TTL thresholds accept a 3.3 V input.

from machine import Pin, PWM
import time

# ---------------------------------------------------------------- configuration

# One pin per motor, or two? A BTL driver of the BA5947FP class takes PWM directly
# with an internal filter, so a single pin per channel carries both direction and
# magnitude: 50% duty is stop, above is one way, below is the other. A plain
# H-bridge (L293D, DRV8833) wants two pins per motor instead.
MODE          = "btl"        # "btl" (1 pin/motor, centre = stop) or "hbridge"

SLED_PINS     = (2, 3)       # btl: (pwm, unused). hbridge: (in1, in2)
SPIN_PINS     = (4, 5)
LIMIT_PIN     = 6            # inner limit switch
LIMIT_ACTIVE_LOW = True      # most are: closed to ground when the sled is home

PWM_HZ        = 25000        # above the driver's internal filter corner, and above
                             # hearing, so the PWM is not itself part of the noise

# The reference outward pulse. Keep it SHORT -- short enough that the sled cannot
# reach the outer end even at full speed. A full stroke is roughly 34 mm and the
# fastest sleds manage about 48 mm/s, so 400 ms moves at most ~19 mm. Verify by eye
# the first time: it must stop well clear of the end.
REF_DUTY      = 0.50
REF_MS        = 400

# Set this from ref_measure(). Until then velocities print as "set REF_MM".
REF_MM        = None

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
    if MODE == "btl":
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
    """Everything off. Safe at any time, including from a KeyboardInterrupt."""
    _coast(_sled)
    _coast(_spin)


def sw():
    """True when the sled is at the inner limit. Use it to check switch polarity."""
    v = _limit.value()
    return (v == 0) if LIMIT_ACTIVE_LOW else (v == 1)


# ------------------------------------------------------------------- primitives

def home(duty=0.45, timeout_ms=MAX_RUN_MS):
    """Inward until the switch asserts. Returns ms taken, or None on timeout.

    The motor is cut the instant the switch reads closed, so it rests against the
    inner stop without being driven into it.
    """
    if sw():
        return 0
    t0 = time.ticks_ms()
    try:
        _drive(_sled, duty, outward=False)
        while not sw():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                print("home: TIMEOUT. The switch never closed. Check LIMIT_ACTIVE_LOW,")
                print("      the switch wiring, and whether 'inward' is really inward.")
                return None
            time.sleep_ms(1)
        return time.ticks_diff(time.ticks_ms(), t0)
    finally:
        stop()


def ref_pulse():
    """The fixed outward hop. Same duty, same time, so the same distance every run."""
    try:
        _drive(_sled, REF_DUTY, outward=True)
        time.sleep_ms(min(REF_MS, MAX_RUN_MS))
    finally:
        stop()


def pulse(duty=0.5, ms=500, outward=True):
    """Drive briefly. For poking at it by hand and for ref_measure()."""
    try:
        _drive(_sled, duty, outward)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


def spin(duty=0.4, ms=2000):
    """Run the spindle, to hear it and confirm the motor pairs are right."""
    try:
        _drive(_spin, duty, outward=True)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


# ----------------------------------------------------------- the one measurement

def ref_measure():
    """Run ONCE, with a ruler. Establishes how far the reference pulse travels.

    Everything else is timing, which the Pico does. This is the single step that
    turns those times into millimetres, and it needs your eyes.
    """
    print("Homing...")
    if home() is None:
        return
    print()
    print("The sled is now at the inner stop.")
    print("MEASURE AND NOTE: pick a fixed datum on the chassis -- an edge, a screw --")
    print("and note where the centre of the lens sits against it. Calipers are ideal,")
    print("a steel rule is enough: you need about 1 mm accuracy over ~15-20 mm.")
    print()
    input("press Enter when you have noted the starting position... ")
    ref_pulse()
    print()
    print("Pulse done (%.2f duty for %d ms)." % (REF_DUTY, REF_MS))
    print("MEASURE AGAIN against the same datum.")
    print()
    print("Check first: is the sled clear of the OUTER end? If it ran into the stop,")
    print("lower REF_MS until it does not -- the whole method depends on this pulse")
    print("landing mid-rail rather than against a stop.")
    print()
    print("Then set the difference at the top of this file:   REF_MM = <mm moved>")
    print("and re-copy it, or just assign it live:            sled_cal.REF_MM = 17.5")


def run_time(duty, settle_ms=250):
    """Home, hop out by the reference distance, then time an inward run at `duty`.

    Returns (ms, mm_per_s), with mm_per_s None until REF_MM is set.
    """
    if home() is None:
        return (None, None)
    time.sleep_ms(settle_ms)
    ref_pulse()
    time.sleep_ms(settle_ms)
    if sw():
        print("run_time: still at the limit after the reference pulse -- it moved")
        print("          nothing, or 'outward' is inverted.")
        return (None, None)
    ms = home(duty)
    if ms is None:
        return (None, None)
    v = (REF_MM * 1000.0 / ms) if REF_MM else None
    return (ms, v)


def cal(duties=(0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.75, 1.00)):
    """The table. One row per duty: time for the reference distance, and velocity.

    Run ref_measure() first, or the velocity column cannot be filled in and you get
    times only -- still useful for finding the stall threshold, just not in mm/s.
    """
    if REF_MM:
        print("reference distance %.1f mm, target %.0f-%.0f mm/s (a Mega CD)\n"
              % (REF_MM, TARGET_LO, TARGET_HI))
        print("  duty    time     velocity   verdict")
        print("  ----  -------  ----------   -------")
    else:
        print("REF_MM is not set, so velocities are unknown -- run ref_measure().")
        print("Times alone still show where it stalls.\n")
        print("  duty    time")
        print("  ----  -------")

    rows = []
    for d in duties:
        ms, v = run_time(d)
        if ms is None:
            print("  %4.2f      ----   stalled, or never reached the switch" % d)
            continue
        if v is None:
            print("  %4.2f  %5d ms" % (d, ms))
        else:
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
        rows.append((d, ms, v))
        time.sleep_ms(400)

    print()
    if not rows:
        print("Nothing moved. Work through, in order:")
        print("  1. sw() must change when you slide the sled by hand")
        print("  2. pulse() must move it -- if not, IC722's mute/standby pins are the")
        print("     prime suspect now the DSP's traces are cut")
        print("  3. if it moves the wrong way, swap SLED_PINS or the motor leads")
        return rows

    if REF_MM:
        inr = [r for r in rows if r[2] and TARGET_LO <= r[2] <= TARGET_HI]
        if inr:
            print("Mega CD velocity is reachable at duty %s."
                  % ", ".join("%.2f" % r[0] for r in inr))
            print("That is the result worth having: a seek becomes ONE continuous sweep")
            print("at the right speed, and the segmentation the USB path needs goes away.")
        else:
            best = min((r for r in rows if r[2]), key=lambda r: r[2], default=None)
            if best:
                print("Nothing landed in %.0f-%.0f mm/s. Slowest was %.1f mm/s at duty %.2f."
                      % (TARGET_LO, TARGET_HI, best[2], best[0]))
                if best[2] > TARGET_HI:
                    print("Static friction is setting the floor. Before concluding, try:")
                    print("  - grease on the rails, or working it back and forth by hand")
                    print("  - PWM_HZ = 2000: the cogging helps break stiction")
                    print("  - a brief kick at high duty, then drop to the low duty")
    else:
        slowest = max(rows, key=lambda r: r[1])
        print("Slowest duty that still completed: %.2f (%d ms)."
              % (slowest[0], slowest[1]))
        print("Run ref_measure() to turn these into mm/s.")
    return rows


def check():
    """Run FIRST. Confirms the switch and both motors before anything is timed."""
    print("switch now: %s" % ("CLOSED (at home)" if sw() else "open (not at home)"))
    print("slide the sled by hand and call sw() again -- it must change.")
    print("if it never changes, fix that before going further.")
    print()
    print("pulsing OUTWARD for 400 ms...")
    pulse(0.5, 400, outward=True)
    out_sw = sw()
    print("  switch now: %s" % ("CLOSED" if out_sw else "open"))
    if out_sw:
        print("  >>> the switch CLOSED after an outward pulse, so the direction is")
        print("  >>> inverted. Swap SLED_PINS or the motor leads before cal().")
    print()
    print("pulsing INWARD for 400 ms...")
    pulse(0.5, 400, outward=False)
    print("  switch now: %s" % ("CLOSED" if sw() else "open"))
    print()
    print("spindle for 2 s -- you should hear it. If the SLED moves instead, the two")
    print("motor pairs are swapped: exchange SLED_PINS and SPIN_PINS.")
    spin(0.4, 2000)
    print()
    print("when the direction and the pairs are right:  ref_measure()  then  cal()")


stop()
print("sled_cal ready.  check()  ->  ref_measure()  ->  cal().  stop() anytime.")
