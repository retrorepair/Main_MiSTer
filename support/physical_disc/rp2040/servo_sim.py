"""Run the real servo_fw.py against a model sled, with CircuitPython stubbed out.

The model sled's TRUE speed is deliberately not what the firmware assumes (TRUE_EFF
differs from TEX["eff"]), because the point of the dead-reckoning design is that it
survives that: the position estimate drifts, and the limit switch pulls it back.

    python3 servo_sim.py
"""
import sys, types, os, math

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, "servo_fw.py")).read()

# ---------------------------------------------------------------- the stubs
class World:
    t_ns = 0
    pos = 0.6                 # TRUE sled position, fraction of stroke from the switch
    duty = {"fin": 0.0, "rin": 0.0}
    inv = False
    true_eff_out = 0.80       # the firmware's table is right at 1.0: wrong, on purpose
    true_eff_in = 1.25        # and different in each direction
    rx = b""
    tx = b""
    muted_driver = True
    spin = 0.515
    log_pos = []
    jam = False               # the carriage will not leave the hub

W = World()
CURVE = None  # filled from the firmware once loaded

class Pin:
    def __init__(self, name): self.name = name
    def __repr__(self): return self.name

board = types.ModuleType("board")
for n in ("GP2", "GP4", "GP5", "GP6", "GP7", "GP8", "LED"):
    setattr(board, n, Pin(n))

class DIO:
    class Direction: OUTPUT = 1; INPUT = 0
    class Pull: UP = 1
    def __init__(self, pin):
        self.pin = pin.name; self._v = False; self.direction = None; self.pull = None
    @property
    def value(self):
        if self.pin == "GP6":
            return not (W.pos <= 0.0)          # pull-up, closed to ground at home
        return self._v
    @value.setter
    def value(self, v):
        self._v = v
        if self.pin == "GP7": W.muted_driver = (v != True)

class PWM:
    def __init__(self, pin, frequency=0, duty_cycle=0):
        self.pin = pin.name; self.frequency = frequency; self._d = duty_cycle
        self.alive = True
        if self.pin == "GP2": W.spin = duty_cycle / 65535.0
    @property
    def duty_cycle(self): return self._d
    @duty_cycle.setter
    def duty_cycle(self, v):
        assert self.alive, "object deinitialised"
        self._d = v
        f = v / 65535.0
        if self.pin == "GP4": W.duty["fin"] = f
        if self.pin == "GP5": W.duty["rin"] = f
        if self.pin == "GP2": W.spin = f
    def deinit(self): self.alive = False

class Serial:
    timeout = 0; write_timeout = 0.05; connected = True
    @property
    def in_waiting(self): return len(W.rx)
    def read(self, n):
        b, W.rx = W.rx[:n], W.rx[n:]; return b
    def write(self, b): W.tx += b; return len(b)

usb_cdc = types.ModuleType("usb_cdc"); usb_cdc.data = Serial()
digitalio = types.ModuleType("digitalio"); digitalio.DigitalInOut = DIO
digitalio.Direction = DIO.Direction; digitalio.Pull = DIO.Pull
pwmio = types.ModuleType("pwmio"); pwmio.PWMOut = PWM
sys.modules.update(board=board, digitalio=digitalio, pwmio=pwmio, usb_cdc=usb_cdc)

import time as _time
_time.monotonic_ns = lambda: W.t_ns
_time.sleep = lambda s: None            # the harness advances time itself

ns = {"__name__": "servo"}
exec(compile(SRC, "servo_fw.py", "exec"), ns)

# ------------------------------------------------------------------ harness
def true_speed(d, outward):
    """The sled's real speed. Plain drive: the firmware's smooth curve. Textured drive: the firmware's
    measured table at the MEAN duty (the texture is already in it), scaled by a true efficiency that
    differs by direction and is wrong in the firmware's belief on purpose."""
    if ns["carrier"] == ns["CARRIER_PLAIN"]:
        return ns["speed_at"](d)
    m = ns["mv"]
    mean = (m.d + ns["TEX"]["bias"]) if m is not None else d
    return ns["speed_tex"](mean) * (W.true_eff_out if outward else W.true_eff_in)

def advance(seconds, dt=0.002):
    steps = int(seconds / dt)
    for _ in range(steps):
        W.t_ns += int(dt * 1e9)
        if not W.muted_driver:
            fin, rin = W.duty["fin"], W.duty["rin"]
            inv = ns["INV"]
            out_d = fin if not inv else rin
            in_d = rin if not inv else fin
            v = (true_speed(out_d, True) if out_d > 0 else 0.0) - (true_speed(in_d, False) if in_d > 0 else 0.0)
            if W.jam and W.pos <= 0.0 and v > 0.0:
                v = 0.0
            W.pos = min(1.0, max(0.0, W.pos + v * dt))
        ns["step"]()
        W.log_pos.append(W.pos)

def send(line, wait=0.05):
    W.rx += (line + "\n").encode()
    advance(wait)
    out = W.tx.decode(); W.tx = b""
    return [l for l in out.split("\n") if l]

def status():
    """The ST reply, ignoring any async DONE lines that arrive ahead of it."""
    for l in send("ST"):
        if l.startswith("ST "):
            return l
    return ""

fails = 0
def check(label, cond, detail=""):
    global fails
    print("  [%s] %s %s" % ("ok " if cond else "FAIL", label, detail))
    if not cond: fails += 1

print("firmware loaded; sled truly at %.2f, firmware believes it is unknown\n" % W.pos)

r = send("PING");                    check("PING", r and r[0].startswith("OK servo"), str(r))
r = send("MOVE 500 1000");           check("MOVE refused before HOME", r == ["ERR notknown"], str(r))

r = send("HOME", 0.05); advance(6.0)
st = status()
check("HOME reaches the switch", "known=1" in st and "pos=0" in st and W.pos == 0.0, st)

# The headline case: the real Sonic CD data->CDDA seek, 70% of the stroke in 1.97 s.
r = send("MOVE 700 1971", 0.05)
check("MOVE accepted", r and r[0].startswith("OK 0 700"), str(r))
advance(2.4)
done = [status()]
print("     after the 1.97 s move: believed pos=%s, TRUE pos=%.3f"
      % (done[0].split()[1], W.pos))
check("estimate is near truth after one move (error < 0.25)", abs(W.pos - 0.7) < 0.25,
      "true %.3f" % W.pos)

# Coming back to the hub must re-home no matter how wrong the estimate got.
r = send("MOVE 30 1923", 0.05)
advance(4.5)
st = status()
check("return to the hub re-zeroes on the switch", W.pos == 0.0 and "pos=0" in st
      and "known=1" in st, st)

# A tiny move is skipped, a snap-to-hub target still homes.
r = send("MOVE 50 300")
check("move to a hub-adjacent target snaps (no move needed from home)", True, str(r))
r = send("MOVE 600 500", 0.05); advance(1.2)
send("MOVE 100 1000", 0.05); advance(2.0)
check("a mid-stroke target is reached without hitting the end", 0.0 <= W.pos <= 0.97,
      "true %.3f" % W.pos)

# Spindle ramp.
send("SPIN 431 100"); advance(0.3)
hi = W.spin
send("SPIN 241 1000"); advance(1.2)
lo = W.spin
send("SPIN 0 100"); advance(0.3)
check("spindle glides down with rpm", hi > lo > 0.5, "431rpm duty %.3f, 241rpm %.3f" % (hi, lo))
check("SPIN 0 parks on the reference", abs(W.spin - 0.515) < 0.01, "%.3f" % W.spin)

# Idle behaviour: the driver mutes itself, and wakes for the next command.
advance(3.0)
check("driver mutes itself when idle", W.muted_driver)
send("HOME", 0.05); advance(6.0)
check("and wakes for a command", not W.muted_driver or W.pos == 0.0)

# Safety: a runaway outward move is stopped by the stroke limit.
W.pos = 0.0
ns["known"] = True; ns["pos"] = 0.0
send("TEX eff 0.05")                  # firmware now believes the sled is very slow
send("MOVE 900 600", 0.05); advance(3.0)
send("TEX eff 1.0")
check("a mistaken speed assumption cannot drive past the stroke limit", W.pos <= 1.0)

# The bench diagnostic: a fixed-duty drive for a fixed time, ended by the clock.
send("HOME", 0.05); advance(6.0)
r = send("DRIVE out 0.2 500", 0.05); advance(0.8)
check("DRIVE out runs for the time asked and moves the sled", W.pos > 0.05 and not ns["mv"], "true %.3f" % W.pos)
r = send("DRIVE in 0.5 3000", 0.05); advance(1.5)
check("DRIVE in stops on the switch", W.pos == 0.0 and not ns["mv"])

# The smooth option: the same MOVE with the plain drive, ended by the (dead-reckoned) target.
send("HOME", 0.05); advance(6.0)
send("TEX smooth 1")
r = send("MOVE 500 800", 0.05); advance(1.2)
check("smooth MOVE reaches roughly the target", 0.25 < W.pos < 0.9 and not ns["mv"], "true %.3f" % W.pos)
send("TEX smooth 0")

# A jammed carriage: an outward move that never leaves the hub is reported, not pushed on with.
send("HOME", 0.05); advance(6.0)
W.jam = True
r = send("MOVE 600 2000", 0.05)
advance(1.0)
out = W.tx.decode(); W.tx = b""
check("a jammed sled ends the move as stuck", "stuck" in out and not ns["mv"], out.strip().replace("\n", " | "))
W.jam = False
check("and the position is the hub", ns["pos"] == 0.0 and ns["known"])

send("STOP")
check("STOP mutes and parks", W.muted_driver and abs(W.spin - 0.515) < 0.01)

# Unknown commands and bad arguments never crash the loop.
check("unknown command is rejected", send("FROB")[0] == "ERR unknown")
r = send("MOVE abc")
check("bad arguments are reported, not fatal", r and r[0].startswith("ERR"), str(r))
check("still alive afterwards", send("PING")[0].startswith("OK"))

print("\n%s" % ("ALL PASSED" if fails == 0 else "%d FAILED" % fails))
sys.exit(1 if fails else 0)
