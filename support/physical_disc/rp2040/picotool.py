#!/usr/bin/env python3
"""Talk to the servo board from a Linux box -- in practice the MiSTer, which has python3 and no
pyserial, when the Pico is plugged into it instead of the PC. Standard library only.

    picotool.py find
    picotool.py send  <data-port> "TEX spin_lo 0.55" "SPIN 241 300" [--wait 0.3]
    picotool.py steps <data-port> 0.53,0.55,0.57 [--on 4] [--off 2]
    picotool.py script <data-port> "SPIN 241 300" @sleep=1 "DRIVE out 0.11 1000" @done "HOME 0.5" @done
    picotool.py deploy <console-port> <file> [--dest /code.py]
    picotool.py grain <data-port> "A:grit=0.06;B:grit=0.12,carrier=300" [--pause 2.5]

CircuitPython exposes two ports. The console (REPL) is the lower /dev/ttyACM number and the data
port (the protocol in servo_fw.py) the higher; `find` tells them apart by PING. Do not run this
while the MiSTer core's rig backend has the data port open.
"""
import argparse, base64, binascii, os, select, sys, termios, time


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    t = termios.tcgetattr(fd)
    t[0] = 0
    t[1] = 0
    t[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    t[3] = 0
    t[4] = t[5] = termios.B115200
    t[6][termios.VMIN] = 0
    t[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, t)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_for(fd, seconds):
    end = time.time() + seconds
    out = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], max(0.0, min(0.05, end - time.time())))
        if r:
            try:
                out += os.read(fd, 4096)
            except BlockingIOError:
                pass
    return out.decode("ascii", "replace")


def cmd_find(args):
    for n in range(8):
        p = "/dev/ttyACM%d" % n
        if not os.path.exists(p):
            continue
        fd = open_raw(p)
        os.write(fd, b"PING\n")
        reply = read_for(fd, 0.6).strip()
        os.close(fd)
        print("%s: %s" % (p, reply or "(no reply: the REPL console)"))


def cmd_send(args):
    fd = open_raw(args.port)
    for line in args.lines:
        os.write(fd, (line + "\n").encode())
        print("%-24s -> %s" % (line, read_for(fd, args.wait).strip().replace("\n", " | ")))
    os.close(fd)


def cmd_steps(args):
    """Spin the spindle at each duty in turn, with a pause between, so a listener can count them."""
    fd = open_raw(args.port)

    def tx(line, wait=0.15):
        os.write(fd, (line + "\n").encode())
        read_for(fd, wait)

    tx("TEX spin_lo 0.55")
    for i, d in enumerate(args.duties.split(","), 1):
        # SPIN's rpm sets the duty linearly between spin_lo (241 rpm) and spin_hi (431 rpm), so pin
        # both ends to the duty under test and ask for 300 rpm.
        tx("TEX spin_lo %s" % d)
        tx("TEX spin_hi %s" % d)
        print("step %d: duty %s" % (i, d), flush=True)
        tx("SPIN 300 600")
        time.sleep(args.on)
        tx("SPIN 0 600")
        time.sleep(args.off)
    tx("STOP")
    os.close(fd)


def wait_done(fd, timeout):
    """Collect lines until a DONE line arrives; return it (or '' on timeout)."""
    end = time.time() + timeout
    buf = ""
    while time.time() < end:
        buf += read_for(fd, 0.05)
        for l in buf.splitlines():
            if l.startswith("DONE"):
                return l
    return ""


def done_ms(line):
    p = line.split()
    return int(p[3]) if len(p) >= 5 else -1


def cmd_grain(args):
    """Play the Mega CD data->CDDA seek and its return once per texture variant, so they can be
    compared by ear, and report how far the sled really travelled (smooth HOME as the ruler: 472 ms is
    a whole stroke) and how long the return took. Variants are NAME:key=value,key=value separated by ;"""
    base = {"carrier": 300, "swell": 4.9, "amp": 0.03, "grit": 0.12, "bias": 0.0}   # servo_fw.py defaults
    fd = open_raw(args.port)

    def tx(line, wait=0.12):
        os.write(fd, (line + "\n").encode())
        return read_for(fd, wait)

    def run(line, timeout):
        os.write(fd, (line + "\n").encode())
        return wait_done(fd, timeout)

    for i, v in enumerate(args.variants.split(";"), 1):
        name, _, kv = v.partition(":")
        t = dict(base)
        for pair in filter(None, kv.split(",")):
            k, _, val = pair.partition("=")
            t[k] = float(val)
        for k, val in t.items():
            tx("TEX %s %s" % (k, val), 0.05)
        print("variant %d (%s): %s" % (i, name, ", ".join("%s=%s" % kv for kv in sorted(t.items()))), flush=True)
        tx("SPIN 431 300", 0.3)
        run("HOME 0.5", 9)
        tx("MOVE 700 1971", 0.1); tx("SPIN 241 1971", 0.05)
        out = wait_done(fd, 5)
        time.sleep(0.4)
        ruler = run("HOME 0.5", 9)
        x = done_ms(ruler) / 472.0
        time.sleep(0.8)
        tx("SPIN 431 300", 0.05)
        tx("MOVE 700 1971", 0.1); tx("SPIN 241 1971", 0.05)
        wait_done(fd, 5)
        time.sleep(0.4)
        tx("MOVE 30 1923", 0.1); tx("SPIN 431 1923", 0.05)
        back = wait_done(fd, 6)
        print("   out %s | true travel %.2f of the stroke (asked 0.70) | back %s ms (asked 1923)"
              % (out.split()[-1] if out else "?", x, done_ms(back)), flush=True)
        time.sleep(args.pause)
    for k, val in base.items():
        tx("TEX %s %s" % (k, val), 0.05)
    tx("SPIN 0 300", 0.4)
    tx("STOP")
    os.close(fd)


def cmd_script(args):
    """Run a list of steps. A plain step is a protocol line (its reply is printed); @sleep=S waits;
    @done waits for the next DONE line and prints it."""
    fd = open_raw(args.port)
    for st in args.steps:
        if st.startswith("@sleep="):
            time.sleep(float(st[7:]))
        elif st == "@done":
            print("   %s" % (wait_done(fd, 12) or "(no DONE)"), flush=True)
        else:
            os.write(fd, (st + "\n").encode())
            print("%-24s -> %s" % (st, read_for(fd, 0.15).strip().replace("\n", " | ")), flush=True)
    os.close(fd)


def cmd_deploy(args):
    data = open(args.file, "rb").read()
    b64 = base64.b64encode(data).decode()
    crc = binascii.crc32(data) & 0xFFFFFFFF
    fd = open_raw(args.port)
    os.write(fd, b"\x03")
    read_for(fd, 0.5)
    os.write(fd, b"\x03")
    read_for(fd, 0.7)
    os.write(fd, b"\r")
    read_for(fd, 0.4)
    cmds = ["import binascii, gc, os", "gc.collect()", "B = b''"]
    for i in range(0, len(b64), 100):
        cmds.append("B += b'%s'" % b64[i:i + 100])
    cmds += ["D = binascii.a2b_base64(B)", "B = None", "gc.collect()",
             "print('DECODED', len(D), hex(binascii.crc32(D)))",
             "g = open('%s', 'wb')" % args.dest, "print('WRITE', g.write(D))", "g.flush()", "g.close()",
             "R = open('%s','rb').read()" % args.dest, "print('NOW', len(R), hex(binascii.crc32(R)))"]
    log = ""
    for c in cmds:
        os.write(fd, (c + "\r").encode())
        log += read_for(fd, 0.12)
    log += read_for(fd, 1.0)
    for l in log.splitlines():
        if any(k in l for k in ("DECODED", "WRITE", "NOW", "Error", "Traceback", "Memory")) and not l.startswith(">>> "):
            print(l)
    print("expected: len %d, crc 0x%x" % (len(data), crc))
    os.write(fd, b"import microcontroller\r")
    read_for(fd, 0.3)
    os.write(fd, b"microcontroller.reset()\r")
    read_for(fd, 0.3)
    os.close(fd)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="c", required=True)
    sub.add_parser("find").set_defaults(f=cmd_find)
    p = sub.add_parser("send"); p.add_argument("port"); p.add_argument("lines", nargs="+")
    p.add_argument("--wait", type=float, default=0.3); p.set_defaults(f=cmd_send)
    p = sub.add_parser("steps"); p.add_argument("port"); p.add_argument("duties")
    p.add_argument("--on", type=float, default=4.0); p.add_argument("--off", type=float, default=2.5)
    p.set_defaults(f=cmd_steps)
    p = sub.add_parser("grain"); p.add_argument("port"); p.add_argument("variants")
    p.add_argument("--pause", type=float, default=2.5); p.set_defaults(f=cmd_grain)
    p = sub.add_parser("script"); p.add_argument("port"); p.add_argument("steps", nargs="+")
    p.set_defaults(f=cmd_script)
    p = sub.add_parser("deploy"); p.add_argument("port"); p.add_argument("file")
    p.add_argument("--dest", default="/code.py"); p.set_defaults(f=cmd_deploy)
    a = ap.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
