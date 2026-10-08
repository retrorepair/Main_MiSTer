"""Run the real code.py against a model sled, with CircuitPython stubbed.

The model matters: the property-vs-method bug that killed the first version could not
be seen by a test that never called at_home(). This one drives the whole of main().
"""
import ast, os, sys, types

SRC = "/mnt/c/t/pd/support/physical_disc/rp2040/code.py"
src = open(SRC).read()
ast.parse(src)

def run_case(label, rin_is_inward, stroke_mm=34.0, mm_per_s_at_full=60.0, pre_dir=None, stuck=False):
    st = dict(t=0.0, pos=stroke_mm, a=0.0, b=0.0, log=[], leds=[])   # pos: mm from the inner switch

    class FakePin:
        pass
    class DIO:
        class Direction: OUTPUT = 1; INPUT = 0
        class Pull: UP = 1
        def __init__(self, pin): self.pin = pin; self._v = False; self.direction = None; self.pull = None
        @property
        def value(self):
            if self.pin == "GP6":
                return not (st["pos"] <= 0.0)      # pull-up, closed to ground when home
            return self._v
        @value.setter
        def value(self, v):
            self._v = v
            if self.pin == "LED": st["leds"].append((round(st["t"], 2), bool(v)))
    class PWM:
        def __init__(self, pin, frequency=0, duty_cycle=0):
            self.pin = pin; self._d = duty_cycle
        @property
        def duty_cycle(self): return self._d
        @duty_cycle.setter
        def duty_cycle(self, v):
            self._d = v
            if self.pin == "GP4": st["a"] = v / 65535.0       # FIN
            if self.pin == "GP5": st["b"] = v / 65535.0       # RIN
    dio = types.ModuleType("digitalio"); dio.DigitalInOut = DIO
    dio.Direction = DIO.Direction; dio.Pull = DIO.Pull
    pw = types.ModuleType("pwmio"); pw.PWMOut = PWM
    bd = types.ModuleType("board")
    for n in ("GP2", "GP4", "GP5", "GP6", "GP7", "GP8", "LED"): setattr(bd, n, n)
    sys.modules.update(digitalio=dio, pwmio=pw, board=bd)

    class Done(Exception): pass
    def sleep(s):
        # advance the sled over this interval
        inward_drive = (st["b"] if rin_is_inward else st["a"]) - (st["a"] if rin_is_inward else st["b"])
        v = inward_drive * mm_per_s_at_full
        if not stuck:
            st["pos"] = min(stroke_mm, max(-5.0, st["pos"] - v * s))
        st["t"] += s
        if st["t"] > 200: raise Done()
    import time
    time.sleep = sleep
    time.monotonic = lambda: st["t"]

    tmp = "/tmp/simfs_%s" % label
    os.makedirs(tmp, exist_ok=True)
    for f in ("results.csv", "dir.txt"):
        try: os.remove(os.path.join(tmp, f))
        except OSError: pass
    if pre_dir is not None:
        open(os.path.join(tmp, "dir.txt"), "w").write(pre_dir)
    code = src.replace('RESULTS     = "/results.csv"', 'RESULTS     = "%s/results.csv"' % tmp)
    code = code.replace('DIRFILE = "/dir.txt"', 'DIRFILE = "%s/dir.txt"' % tmp)
    code = code.replace("\nmain()\n", "\n")
    ns = {"__name__": "code"}
    exec(compile(code, "code.py", "exec"), ns)
    outcome = "returned"
    try:
        ns["main"]()
    except Done:
        outcome = "reached the idle/blink loop"
    except Exception as e:
        outcome = "CRASHED: %s: %s" % (type(e).__name__, e)
    d = open(os.path.join(tmp, "dir.txt")).read() if os.path.exists(os.path.join(tmp, "dir.txt")) else "(none)"
    r = open(os.path.join(tmp, "results.csv")).read().strip().splitlines() if os.path.exists(os.path.join(tmp, "results.csv")) else []
    print("%-52s -> %s | dir.txt=%s | results rows=%d | sled ended at %.1f mm"
          % (label, outcome, d, max(0, len(r) - 1) if r and r[0].startswith("duty") else len(r), st["pos"]))
    return st, r

print("code.py parses OK\n")
run_case("1. first power-up, RIN is inward", True)
run_case("2. first power-up, FIN is inward (wrong guess)", False)
run_case("3. first power-up, sled cannot move (not driven)", True, stuck=True)
# After detection the sled is at the switch; wind out is simulated by pos reset.
st, r = run_case("4. later power-up, dir=0, sled wound out", True, pre_dir="0")
print("   results.csv:", r)
