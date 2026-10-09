# servo_fw.py -- noise-rig firmware for the PS1 CD mechanism. CircuitPython, RP2040.
#
# Deploy as /code.py on the CIRCUITPY drive, with the matching boot.py (which enables
# the second USB serial port this talks on). The host -- the MiSTer, or a script --
# sends one ASCII command per line over that port and gets one reply line back:
#
#   PING                    -> OK servo 1
#   ST                      -> ST pos=<permille> known=<0|1> moving=<0|1> ...
#   HOME [duty]             -> OK, then DONE HOME <ms> <reason>   (inward to the switch)
#   MOVE <permille> <ms>    -> OK <from> <to>, then DONE MOVE <pos> <ms> <reason>
#   DRIVE <out|in> <duty> <ms> -> OK, then DONE MOVE ...   (bench diagnostic, fixed duty)
#   SPIN <rpm> <ms>         -> OK         (0 rpm parks the spindle)
#   TEX <name> <value>      -> OK         (texture/tuning parameters, see TEX below)
#   MUTE <0|1>              -> OK         (1 disables the driver chip)
#   DIR <0|1>               -> OK         (polarity, saved)
#   STOP                    -> OK         (everything off, driver muted)
#   RESET                   -> restarts the board
#
# Positions are permille of the sled's stroke measured from the inner limit switch:
# 0 is the switch, 1000 is the outer stop. The host sends ABSOLUTE targets and a
# duration; this decides the drive strength from the bench-measured speed curve. There
# is no position sensor except the switch, so position is dead-reckoned and re-zeroed
# every time the sled reaches the switch. Targets near the hub snap to the switch for
# exactly that reason: every return to the data area re-homes.
#
# Wiring (BA5977FP, IC722, from Sony's service-manual schematic):
#   GP4 -> pin 23 (ch3 FIN)   GP5 -> pin 22 (ch3 RIN)    the sled
#   GP7 -> pin 20 (MUTE, high = run)   GP8 -> pin 3 (SW, held low)
#   GP2 -> RC -> pin 24 (ch4IN)        the spindle
#   GP6 -> limit switch to GND

VERSION = "servo 1"

import board
import digitalio
import pwmio
import time
import math
import random
import usb_cdc

PIN_FIN, PIN_RIN = board.GP4, board.GP5
PIN_LIMIT = board.GP6
PIN_MUTE = board.GP7
PIN_SW = board.GP8
PIN_SPIN = board.GP2

RUN_HIGH = True              # MUTE pin level that means "driver running"
LIMIT_CLOSED_LOW = True
SPIN_STOP_DUTY = 0.515       # on the reference (Sony shows OUTVref at 1.7 V = 1.7 / 3.3)
CARRIER_PLAIN = 25000        # inaudible: used wherever no texture is wanted

# Full-stroke time against duty for the SMOOTH drive (25 kHz PWM), measured on the bench from
# the outer stop to the inner switch, repeatable to 1-5%. Below 0.16 the sled stalls on stiction.
# Only the plain homing run uses this; see SERVO_RIG.md.
CURVE = ((0.16, 5.04), (0.18, 3.30), (0.20, 2.40), (0.22, 1.84), (0.25, 1.375),
         (0.30, 0.949), (0.35, 0.777), (0.40, 0.675), (0.50, 0.510), (0.60, 0.416),
         (0.75, 0.332), (1.00, 0.253))
SPEEDS = [(d, 1.0 / t) for d, t in CURVE]

# The TEXTURED drive is a different animal. Its 440 Hz carrier gives the motor full-voltage pulses
# that break the stiction, so it moves the sled at duties where the smooth drive stalls, and
# faster than the smooth curve above at the same mean duty. Measured on the bench with DRIVE and
# the HOME ruler (driveprobe.ps1), outward and inward agree to a few percent: mean duty
# (before the swell and grit) against strokes of the PHYSICAL stroke per second, with the default
# carrier/swell/amp/grit. Change those and this table no longer holds.
TEX_CURVE = ((0.02, 0.01), (0.04, 0.065), (0.06, 0.138), (0.08, 0.243), (0.10, 0.338),
             (0.12, 0.455), (0.14, 0.56), (0.16, 0.68), (0.18, 0.78), (0.20, 0.88),
             (0.24, 1.04), (0.30, 1.36), (0.40, 1.84), (0.50, 2.30), (1.00, 4.0))

# Tunables, changed with TEX. The defaults are the sound the owner picked by ear (P).
TEX = {
    "carrier": 300.0,   # Hz of the grind; below ~1 kHz it is audible and rough
    "swell": 4.9,       # loud/quiet cycles per second
    "amp": 0.03,        # swell depth, in duty
    "grit": 0.12,       # random duty noise
    "bias": 0.0,        # added to every textured duty
    "kick_ms": 12.0,    # smooth moves start with a kick: the datasheet's sled kick (CXD2545Q p.30) is
    "kick_gain": 2.0,   # basic level x1..x4 for 2.9..23.2 ms. The levels here are a first guess
    "brake_ms": 10.0,   # until a real PS1 is captured; the brake is the reverse pulse that ends a jump
    "smooth_min_v": 0.25,  # the smooth drive stalls below ~0.2 strokes/s, so slower moves stay textured
    "eff_smooth": 1.23,     # the smooth drive's real speed against the table, measured outward with a
                            # 620 ms move: 0.63 of the stroke travelled for 0.52 asked
    "eff_smooth_in": 1.07,  # and inward, extra: 0.63 came home in 560 ms for 600 asked
    "smooth": 0.0,      # 1 = move the sled with the plain 25 kHz drive (a clean whirr, no grind): what a
                        # PlayStation or Saturn sled sounds like, against the Mega CD's grimy drag
    "eff": 1.04,        # scales the textured speed the firmware believes. From six repeats of
                        # MOVE 700 1971 with this texture: 0.73 of the stroke travelled, 0.70 asked.
    "eff_in": 1.14,     # the same, extra, for INWARD moves: the sled comes home about 19% faster
                        # than the table (1.68 s against 1.92 s asked, +-5% run to run)
    "min": 0.02,        # moves shorter than this fraction of the stroke are skipped
    "snap": 0.08,       # targets below this snap to the switch
    "spin_lo": 0.57,    # spindle duty at 241 rpm (rim). By ear on the bench: steps 0.53-0.62
    "spin_hi": 0.61,    # were smooth, 0.66 and 0.70 were loud and sounded like something hitting,
                        # so the whole range stays under 0.62. Not measured against real rpm.
    "idle_mute": 1.5,   # seconds of stillness before the driver is muted
}
RPM_LO, RPM_HI = 241.0, 431.0


def u16(f):
    if f < 0.0:
        f = 0.0
    elif f > 1.0:
        f = 1.0
    return int(f * 65535)


def speed_at(d):
    """Strokes per second at this duty, from the measured curve."""
    if d <= 0.14:
        return 0.0
    first = SPEEDS[0]
    if d < first[0]:
        return first[1] * (d - 0.14) / (first[0] - 0.14)
    if d >= SPEEDS[-1][0]:
        return SPEEDS[-1][1]
    for i in range(len(SPEEDS) - 1):
        d0, s0 = SPEEDS[i]
        d1, s1 = SPEEDS[i + 1]
        if d0 <= d <= d1:
            f = (d - d0) / (d1 - d0)
            return math.exp(math.log(s0) + f * (math.log(s1) - math.log(s0)))
    return SPEEDS[-1][1]


def speed_tex(d):
    """Strokes per second of the TEXTURED drive at this mean duty, from TEX_CURVE."""
    c = TEX_CURVE
    if d <= c[0][0]:
        return c[0][1] * (d / c[0][0]) if d > 0.0 else 0.0
    for i in range(len(c) - 1):
        d0, s0 = c[i]
        d1, s1 = c[i + 1]
        if d <= d1:
            return s0 + (s1 - s0) * (d - d0) / (d1 - d0)
    return c[-1][1]


def duty_for_plain(v):
    """The duty that gives the SMOOTH drive v strokes per second (it stalls below about 0.15)."""
    lo, hi = 0.15, 1.0
    for _ in range(22):
        mid = (lo + hi) / 2.0
        if speed_at(mid) < v:
            lo = mid
        else:
            hi = mid
    return hi


def duty_for(v):
    """The mean duty that gives the textured sled v strokes per second."""
    c = TEX_CURVE
    if v <= c[0][1]:
        return c[0][0]
    for i in range(len(c) - 1):
        d0, s0 = c[i]
        d1, s1 = c[i + 1]
        if v <= s1:
            return d0 + (d1 - d0) * (v - s0) / (s1 - s0)
    return c[-1][0]


# ------------------------------------------------------------------- hardware

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
limit = digitalio.DigitalInOut(PIN_LIMIT)
limit.direction = digitalio.Direction.INPUT
limit.pull = digitalio.Pull.UP
mute = digitalio.DigitalInOut(PIN_MUTE)
mute.direction = digitalio.Direction.OUTPUT
mute.value = not RUN_HIGH              # start muted: nothing can move until asked
sw = digitalio.DigitalInOut(PIN_SW)
sw.direction = digitalio.Direction.OUTPUT
sw.value = False
muted = True

carrier = CARRIER_PLAIN
fin = pwmio.PWMOut(PIN_FIN, frequency=carrier, duty_cycle=0)
rin = pwmio.PWMOut(PIN_RIN, frequency=carrier, duty_cycle=0)
spin = pwmio.PWMOut(PIN_SPIN, frequency=CARRIER_PLAIN, duty_cycle=u16(SPIN_STOP_DUTY))


def load_dir():
    try:
        with open("/dir.txt") as f:
            return f.read().strip() == "1"
    except OSError:
        return False


INV = load_dir()


def set_carrier(f):
    global carrier, fin, rin
    f = int(f)
    if f == carrier:
        return
    fin.deinit()
    rin.deinit()
    fin = pwmio.PWMOut(PIN_FIN, frequency=f, duty_cycle=0)
    rin = pwmio.PWMOut(PIN_RIN, frequency=f, duty_cycle=0)
    carrier = f


def drive(outward, d):
    # Which input carries the PWM for each direction was found on the bench and is
    # stored as INV; see dir.txt.
    if outward != INV:
        fin.duty_cycle = u16(d)
        rin.duty_cycle = 0
    else:
        fin.duty_cycle = 0
        rin.duty_cycle = u16(d)


def coast():
    fin.duty_cycle = 0
    rin.duty_cycle = 0


def at_home():
    return (not limit.value) if LIMIT_CLOSED_LOW else limit.value


# ---------------------------------------------------------------------- state

pos = 0.0            # dead-reckoned, fraction of the stroke from the switch
known = False        # True once the switch has been reached since power-up
mv = None            # the move in progress
spin_ramp = None     # (t0_ns, T_s, duty0, duty1)
spin_rpm = 0.0       # the rpm the host last asked for
spin_duty = SPIN_STOP_DUTY
last_active = time.monotonic_ns()

ser = usb_cdc.data
if ser is not None:
    ser.timeout = 0
    try:
        ser.write_timeout = 0.05
    except Exception:
        pass
rxbuf = b""


def say(s):
    if ser is None:
        return
    try:
        ser.write((s + "\n").encode())
    except Exception:
        pass


def wake():
    """Enable the driver before anything moves."""
    global muted, last_active
    last_active = time.monotonic_ns()
    if muted:
        mute.value = RUN_HIGH
        muted = False
        time.sleep(0.03)


def set_mute(on):
    global muted
    mute.value = (not RUN_HIGH) if on else RUN_HIGH
    muted = bool(on)


class Mv:
    pass


def start_move(kind, outward, target, d_cmd, T, plain, limit_s, k=1.0):
    global mv
    m = Mv()
    m.kind, m.outward, m.target, m.d = kind, outward, target, d_cmd
    m.T, m.plain, m.limit_s = T, plain, limit_s
    m.k = k                    # real speed over the planned speed, for dead reckoning
    m.t0 = time.monotonic_ns()
    m.last = m.t0
    m.p0 = pos
    set_carrier(CARRIER_PLAIN if plain else TEX["carrier"])
    mv = m


def finish(reason):
    global mv, known, pos
    m = mv
    mv = None
    if reason == "target" and m.plain and m.kind == "move":
        # The jump ends with a reverse kick (CXD2545Q p.57-61: brake B / kick D) before the
        # servo takes over, which is the click at the end of a PlayStation seek.
        drive(not m.outward, m.d)
        time.sleep(TEX["brake_ms"] / 1000.0)
    coast()
    ms = int((time.monotonic_ns() - m.t0) / 1000000)
    if reason == "time" and (m.kind == "home" or m.target <= 0.0):
        known = False             # it never reached the switch: position is unknown
    if reason == "home":
        pos = 0.0
        known = True
    say("DONE %s %d %d %s" % ("HOME" if m.kind == "home" else "MOVE", int(pos * 1000), ms, reason))


# ------------------------------------------------------------------- commands

def cmd_home(args):
    global mv
    if mv is not None:
        coast()
        mv = None
    wake()
    if args:
        d = float(args[0])
        start_move("home", False, -1.0, d, 8.0, True, 9.0)
    else:
        v = 0.5
        start_move("home", False, -1.0, duty_for(v / (TEX["eff"] * TEX["eff_in"])), 2.0, False, 9.0)
    say("OK")


def cmd_move(args):
    global mv
    if not known:
        say("ERR notknown")
        return
    target = float(args[0]) / 1000.0
    T = max(float(args[1]), 50.0) / 1000.0
    if target > 0.95:
        target = 0.95
    if target < TEX["snap"]:
        target = 0.0
    dist = abs(target - pos)
    rehome = (target == 0.0 and pos > 0.02)
    if dist < TEX["min"] and not rehome:
        say("OK skip")
        return
    if mv is not None:
        coast()
        mv = None
    wake()
    outward = target > pos
    v = dist / T
    plain = TEX["smooth"] >= 0.5 and v >= TEX["smooth_min_v"]
    k = 1.0
    if plain:
        k = TEX["eff_smooth"] * (1.0 if outward else TEX["eff_smooth_in"])
        d = duty_for_plain(v / k)
    else:
        d = duty_for(v / (TEX["eff"] * (1.0 if outward else TEX["eff_in"])))
    limit_s = T + (3.0 if target == 0.0 else 0.4)
    start_move("move", outward, target, d, T, plain, limit_s, k)
    say("OK %d %d" % (int(pos * 1000), int(target * 1000)))


def cmd_drive(args):
    """DRIVE <out|in> <duty> <ms>: textured drive at a fixed duty for a fixed time.

    A bench diagnostic, not used in playback: it is how the real speed of the textured
    sled is measured against duty, with HOME as the ruler afterwards. It still stops at
    the switch going in, and at the believed stroke limit going out."""
    global mv
    outward = args[0].lower().startswith("o")
    d = float(args[1])
    T = max(float(args[2]), 20.0) / 1000.0
    if mv is not None:
        coast()
        mv = None
    wake()
    start_move("drive", outward, 1.0 if outward else 0.5, d, T, False, T)
    say("OK")


def rpm_duty(rpm):
    if rpm <= 0.0:
        return SPIN_STOP_DUTY
    f = (rpm - RPM_LO) / (RPM_HI - RPM_LO)
    d = TEX["spin_lo"] + f * (TEX["spin_hi"] - TEX["spin_lo"])
    # Capped under the 0.66 where the bench spindle got loud, so a 2x profile's high rpm
    # (PSX, Saturn) cannot push past it.
    return min(max(d, 0.52), 0.62)


def cmd_spin(args):
    global spin_ramp, spin_rpm
    rpm = float(args[0])
    T = max(float(args[1]), 1.0) / 1000.0 if len(args) > 1 else 0.05
    if rpm > 0.0:
        wake()
    spin_rpm = rpm
    spin_ramp = (time.monotonic_ns(), T, spin_duty, rpm_duty(rpm))
    say("OK")


def cmd_stop():
    global mv, spin_ramp, spin_rpm, spin_duty
    mv = None
    coast()
    spin_ramp = None
    spin_rpm = 0.0
    spin_duty = SPIN_STOP_DUTY
    spin.duty_cycle = u16(SPIN_STOP_DUTY)
    set_mute(True)
    say("OK")


def handle(line):
    global INV
    parts = line.split()
    if not parts:
        return
    c = parts[0].upper()
    a = parts[1:]
    try:
        if c == "PING":
            say("OK " + VERSION)
        elif c == "ST":
            say("ST pos=%d known=%d moving=%d home=%d muted=%d spin=%d inv=%d"
                % (int(pos * 1000), 1 if known else 0, 1 if mv else 0,
                   1 if at_home() else 0, 1 if muted else 0, int(spin_rpm), 1 if INV else 0))
        elif c == "HOME":
            cmd_home(a)
        elif c == "MOVE":
            cmd_move(a)
        elif c == "DRIVE":
            cmd_drive(a)
        elif c == "SPIN":
            cmd_spin(a)
        elif c == "TEX":
            TEX[a[0]] = float(a[1])
            say("OK")
        elif c == "MUTE":
            set_mute(a[0] == "1")
            say("OK")
        elif c == "DIR":
            INV = (a[0] == "1")
            try:
                with open("/dir.txt", "w") as f:
                    f.write("1" if INV else "0")
            except OSError:
                pass
            say("OK")
        elif c == "STOP":
            cmd_stop()
        elif c == "RESET":
            import microcontroller
            microcontroller.reset()
        else:
            say("ERR unknown")
    except Exception as e:
        coast()
        say("ERR " + str(e))


def poll_serial():
    global rxbuf
    if ser is None:
        return
    n = ser.in_waiting
    if n:
        rxbuf += ser.read(n)
        while b"\n" in rxbuf:
            line, rxbuf = rxbuf.split(b"\n", 1)
            try:
                handle(line.decode().strip())
            except Exception:
                say("ERR decode")
        if len(rxbuf) > 256:
            rxbuf = b""


# ----------------------------------------------------------------------- loop

def step():
    global mv, pos, spin_ramp, spin_duty, last_active
    poll_serial()
    ns = time.monotonic_ns()

    m = mv
    if m is not None:
        last_active = ns
        el = (ns - m.t0) * 1e-9
        dt = (ns - m.last) * 1e-9
        m.last = ns
        if m.plain:
            d = m.d
            if m.kind == "move" and el * 1000.0 < TEX["kick_ms"]:
                d = min(1.0, m.d * TEX["kick_gain"])
        else:
            env = 0.5 * (1.0 + math.sin(6.2831853 * TEX["swell"] * el))
            d = (m.d + TEX["bias"] - TEX["amp"] + 2.0 * TEX["amp"] * env
                 + TEX["grit"] * (random.random() * 2.0 - 1.0))
        drive(m.outward, d)
        if m.plain:
            v = speed_at(m.d) * m.k
        else:
            v = speed_tex(m.d + TEX["bias"]) * TEX["eff"] * (1.0 if m.outward else TEX["eff_in"])
        pos += (v * dt) if m.outward else -(v * dt)
        if pos < 0.0:
            pos = 0.0
        end = None
        if (not m.outward) and at_home():
            end = "home"
        elif m.kind == "move" and m.target > 0.0 and (
                (m.outward and pos >= m.target) or ((not m.outward) and pos <= m.target)):
            end = "target"
        elif m.outward and pos >= 0.97:
            end = "limit"
        elif el >= m.limit_s:
            end = "time"
        if end:
            finish(end)
        led.value = True
    else:
        led.value = False

    r = spin_ramp
    if r is not None:
        f = (ns - r[0]) * 1e-9 / r[1]
        if f >= 1.0:
            f = 1.0
            spin_ramp = None
        spin_duty = r[2] + (r[3] - r[2]) * f
        spin.duty_cycle = u16(spin_duty)
        last_active = ns

    # Silence the driver when nothing is going on: the parked spindle input creeps.
    if (not muted) and mv is None and spin_rpm <= 0.0 and spin_ramp is None:
        if (ns - last_active) * 1e-9 > TEX["idle_mute"]:
            set_mute(True)


def main():
    say("BOOT " + VERSION)
    while True:
        step()
        time.sleep(0.002)


if __name__ == "__main__":
    if ser is None:
        # boot.py did not enable the data port: nothing to talk on. Say so on the LED.
        while True:
            led.value = not led.value
            time.sleep(0.1)
    main()
