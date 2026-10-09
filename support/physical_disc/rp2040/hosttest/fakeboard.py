"""A fake servo board on a pty, in real time, for testing physical_disc_rig.cpp on a PC.

Speaks the same protocol as servo_fw.py and logs every command with a timestamp, so the host side's
sequencing and timing can be checked without the hardware. It does not model the sled beyond
"a MOVE takes the time asked, a HOME takes 0.6 s".
"""
import os, sys, time, threading, pty, tty

m, s = pty.openpty()
tty.setraw(m)
name = os.ttyname(s)
link = "/tmp/fakeacm"
try: os.unlink(link)
except OSError: pass
os.symlink(name, link)
print("fake board on", link, flush=True)

t0 = time.time()
known = False
pos = 0
lock = threading.Lock()
log = open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/fakeboard.log", "w", buffering=1)

def out(line):
    with lock:
        os.write(m, (line + "\n").encode())

def note(msg):
    log.write("%7.3f %s\n" % (time.time() - t0, msg))

def later(delay, line):
    def f():
        time.sleep(delay)
        out(line)
    threading.Thread(target=f, daemon=True).start()

buf = b""
while True:
    try:
        d = os.read(m, 256)
    except OSError:
        time.sleep(0.01); continue
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        line = line.decode().strip()
        if not line: continue
        note("<< " + line)
        p = line.split()
        c = p[0]
        if c == "PING": out("OK servo 1")
        elif c == "STOP": known = known; out("OK")
        elif c == "SPIN" or c == "TEX": out("OK")
        elif c == "HOME":
            out("OK"); known = True; pos = 0
            later(0.6, "DONE HOME 0 600 home")
        elif c == "MOVE":
            if not known: out("ERR notknown"); continue
            tgt = int(p[1]); T = int(p[2])
            out("OK %d %d" % (pos, tgt))
            later(T / 1000.0, "DONE MOVE %d %d %s" % (tgt, T, "home" if tgt < 80 else "target"))
            pos = tgt
        else: out("ERR unknown")
