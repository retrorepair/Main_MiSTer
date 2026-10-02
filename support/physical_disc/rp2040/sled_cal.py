# sled_cal.py -- characterise a CD sled for acoustic mirroring
#
# MicroPython for RP2040. Copy to the Pico (as main.py, or paste at the REPL).
# MicroPython rather than the C SDK on purpose: this is a measuring instrument, not
# the final firmware, and drag-and-drop beats standing up a toolchain.
#
# The question it exists to answer: does the sled run SMOOTHLY at 16-21 mm/s, which
# is a Mega CD's sled velocity, or does static friction stall it first? Everything
# else depends on that answer, so measure it before building any more.
#
#
# THE METHOD
#
#   The inner limit switch is the only position feedback, so only INWARD runs can
#   tell us they have arrived. Every measurement is therefore:
#
#       sled at the OUTER stop  ->  drive inward at duty D  ->  switch closes
#
#   The outer stop is a hard mechanical reference, so the distance is the full
#   stroke every time and the only variable is D. Wind it out by hand if you can
#   reach the worm -- nothing stalls that way -- or use out_to_stop().
#
#   PWM duty does NOT tell you velocity: that relationship is what we are measuring,
#   and it depends on the motor, the gearing and the friction. So ONE physical
#   measurement is needed, of the full stroke in mm. stroke_measure() walks through
#   it. Being a ~34 mm measurement rather than a few mm, a steel rule is plenty.
#
#   Results accumulate in /sled_cal.csv and survive a power cycle, so runs can be
#   done one at a time with winding in between. table() prints what you have.
#
#   Getting the file onto a PC: MicroPython does not expose its filesystem as a USB
#   drive (the mass-storage device you see is the UF2 bootloader, which cannot read
#   the Python filesystem). Either copy table()'s output off the REPL, or pull the
#   real file with:   mpremote fs cp :sled_cal.csv .
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
import time, json, os

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

MAX_RUN_MS    = 8000         # hard cap on any single motor run
STATE         = "/sled_cal.csv"
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


# ------------------------------------------------------------------- persistence

def _load():
    """Returns (stroke_mm_or_None, [(duty, ms), ...]) from flash."""
    stroke, rows = None, []
    try:
        with open(STATE) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                k, _, v = line.partition(",")
                if k == "stroke":
                    stroke = float(v)
                elif k == "run":
                    d, _, ms = v.partition(",")
                    rows.append((float(d), int(ms)))
    except OSError:
        pass
    return stroke, rows


def _append(line):
    with open(STATE, "a") as f:
        f.write(line + "\n")


def clear():
    """Throw away the accumulated table and the stroke measurement."""
    try:
        os.remove(STATE)
        print("cleared %s" % STATE)
    except OSError:
        print("nothing to clear")


# ------------------------------------------------------------------- primitives

def home(duty=0.45, timeout_ms=MAX_RUN_MS):
    """Inward until the switch closes. Returns ms taken, or None on timeout.

    The motor is cut the instant the switch reads closed, so the sled rests against
    the inner stop rather than being driven into it.
    """
    if sw():
        return 0
    t0 = time.ticks_ms()
    try:
        _drive(_sled, duty, outward=False)
        while not sw():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                print("home: TIMEOUT, the switch never closed.")
                print("      Check LIMIT_ACTIVE_LOW, the switch wiring, and whether")
                print("      'inward' is really inward. Or the duty is below stall.")
                return None
            time.sleep_ms(1)
        return time.ticks_diff(time.ticks_ms(), t0)
    finally:
        stop()


def out(duty=0.40, ms=300):
    """One bounded outward hop. For walking the sled out in steps, watching it."""
    try:
        _drive(_sled, duty, outward=True)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


def out_to_stop(duty=0.35, ms=4000):
    """Drive outward until it reaches the stop, then stall there for the remainder.

    Only if you cannot reach the worm to wind by hand. Keep `ms` just long enough:
    the motor is stalled for whatever time is left over. Low duty keeps that mild,
    and it is the same condition every CD player homes into at its inner stop.
    """
    print("driving out at duty %.2f for %d ms (will stall briefly at the end)" % (duty, ms))
    try:
        _drive(_sled, duty, outward=True)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()
    print("at the outer stop (assuming ms was long enough -- check by eye once)")


def spin(duty=0.4, ms=2000):
    """Run the spindle, to hear it and confirm the motor pairs are right."""
    try:
        _drive(_spin, duty, outward=True)
        time.sleep_ms(min(ms, MAX_RUN_MS))
    finally:
        stop()


# ------------------------------------------------------------------- measurement

def stroke_measure():
    """Run ONCE, with a rule. The single measurement the Pico cannot make itself.

    Everything else here is timing. This is what turns those times into millimetres.
    """
    print("1. Get the sled to the OUTER end -- wind it by hand, or out_to_stop().")
    input("   press Enter when it is there... ")
    print()
    print("2. Pick a fixed datum on the chassis (an edge, a screw head) and note")
    print("   where the centre of the lens sits against it.")
    input("   press Enter when noted... ")
    print()
    print("3. Homing...")
    if home() is None:
        return
    print("   at the inner stop. Measure against the same datum.")
    print()
    v = input("   full stroke, in mm (blank to abort): ").strip()
    if not v:
        print("   aborted")
        return
    try:
        mm = float(v)
    except ValueError:
        print("   not a number")
        return
    if mm < 5.0 or mm > 60.0:
        print("   %.1f mm is implausible for a CD sled (expect roughly 30-40)" % mm)
        return
    _append("stroke,%.2f" % mm)
    print("   stored: stroke = %.1f mm" % mm)
    print()
    print("Now, for each duty: get the sled to the outer end, then run(<duty>).")
    print("Try 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.75, 1.00.")


def run(duty):
    """Time one full inward stroke at `duty`, and append it to the table.

    The sled must be AT THE OUTER STOP before calling this -- that is what makes
    the distance the same every time.
    """
    if sw():
        print("run: the sled is at the INNER limit. Wind it out to the outer stop")
        print("     first (or out_to_stop()), otherwise there is nothing to time.")
        return None
    stroke, rows = _load()
    ms = home(duty)
    if ms is None:
        print("run: duty %.2f did not reach the switch -- stalled, or too slow." % duty)
        _append("run,%.2f,0" % duty)
        return None
    _append("run,%.2f,%d" % (duty, ms))
    if stroke:
        v = stroke * 1000.0 / ms
        print("duty %.2f: %d ms over %.1f mm = %.1f mm/s" % (duty, ms, stroke, v))
        if TARGET_LO <= v <= TARGET_HI:
            print("  *** IN RANGE for a Mega CD ***")
    else:
        print("duty %.2f: %d ms  (run stroke_measure() to get mm/s)" % (duty, ms))
    # Same duty twice with very different times means the starting point moved.
    same = [r[1] for r in rows if abs(r[0] - duty) < 0.001 and r[1] > 0]
    if same:
        prev = sum(same) / len(same)
        if ms > prev * 1.25 or ms < prev * 0.8:
            print("  NOTE: %d ms against %.0f ms previously at this duty -- the sled"
                  % (ms, prev))
            print("        probably did not start from the same place.")
    return ms


def table():
    """Print everything accumulated so far, and what it means."""
    stroke, rows = _load()
    if not rows:
        print("no runs yet. check() -> stroke_measure() -> run(<duty>)")
        return
    # Average repeats of the same duty.
    byduty = {}
    for d, ms in rows:
        byduty.setdefault(d, []).append(ms)

    print("stroke: %s" % ("%.1f mm" % stroke if stroke else "NOT MEASURED"))
    print("target: %.0f-%.0f mm/s (a Mega CD)" % (TARGET_LO, TARGET_HI))
    print()
    if stroke:
        print("  duty   runs    time     velocity   verdict")
        print("  ----   ----  -------  ----------   -------")
    else:
        print("  duty   runs    time")
        print("  ----   ----  -------")

    inr = []
    for d in sorted(byduty):
        good = [m for m in byduty[d] if m > 0]
        n = len(byduty[d])
        if not good:
            print("  %4.2f   %3d      ----   stalled" % (d, n))
            continue
        ms = sum(good) / len(good)
        if not stroke:
            print("  %4.2f   %3d  %5.0f ms" % (d, n, ms))
            continue
        v = stroke * 1000.0 / ms
        if v < TARGET_LO * 0.6:
            verdict = "too slow"
        elif v < TARGET_LO:
            verdict = "slow side"
        elif v <= TARGET_HI:
            verdict = "*** IN RANGE ***"
            inr.append(d)
        elif v <= TARGET_HI * 1.6:
            verdict = "fast side"
        else:
            verdict = "too fast"
        print("  %4.2f   %3d  %5.0f ms  %6.1f mm/s   %s" % (d, n, ms, v, verdict))

    print()
    if not stroke:
        print("Run stroke_measure() to turn these into mm/s.")
    elif inr:
        print("Mega CD velocity is reachable at duty %s."
              % ", ".join("%.2f" % d for d in inr))
        print("That is the result worth having: a seek becomes ONE continuous sweep")
        print("at the right speed, and the segmentation the USB path needs goes away.")
    else:
        vs = [(d, stroke * 1000.0 / (sum(m for m in byduty[d] if m > 0) /
              len([m for m in byduty[d] if m > 0])))
              for d in byduty if any(m > 0 for m in byduty[d])]
        if vs:
            d, v = min(vs, key=lambda r: r[1])
            print("Nothing in %.0f-%.0f mm/s yet. Slowest was %.1f mm/s at duty %.2f."
                  % (TARGET_LO, TARGET_HI, v, d))
            if v > TARGET_HI:
                print("Static friction is setting the floor. Before concluding, try:")
                print("  - grease on the rails, or working it back and forth by hand")
                print("  - PWM_HZ = 2000: the cogging helps break stiction")
                print("  - a brief kick at high duty, then drop to the test duty")


def check():
    """Run FIRST. Confirms the switch and both motors before anything is timed."""
    print("switch now: %s" % ("CLOSED (at home)" if sw() else "open (not at home)"))
    print("slide or wind the sled and call sw() again -- it must change.")
    print("if it never changes, fix that before anything else.")
    print()
    print("hopping OUTWARD 300 ms...")
    out(0.5, 300)
    if sw():
        print("  switch CLOSED after an OUTWARD hop, so the direction is inverted.")
        print("  swap SLED_PINS, or the motor leads, before measuring anything.")
    else:
        print("  switch open, as expected")
    print()
    print("spindle for 2 s -- you should hear it. If the SLED moves instead, the")
    print("two motor pairs are swapped: exchange SLED_PINS and SPIN_PINS.")
    spin(0.4, 2000)
    print()
    print("then:  stroke_measure()  then  run(0.25), run(0.30), ...  then  table()")


stop()
_s, _r = _load()
print("sled_cal ready.  check() -> stroke_measure() -> run(duty) -> table()")
if _r:
    print("  %d run(s) already stored%s. stop() anytime, clear() to reset."
          % (len(_r), ", stroke %.1f mm" % _s if _s else ""))
