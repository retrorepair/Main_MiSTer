#!/usr/bin/env python3
"""Talk to the servo board from a Linux box -- in practice the MiSTer, which has python3 and no
pyserial, when the Pico is plugged into it instead of the PC. Standard library only.

    picotool.py find
    picotool.py send  <data-port> "TEX spin_lo 0.55" "SPIN 241 300" [--wait 0.3]
    picotool.py steps <data-port> 0.53,0.55,0.57 [--on 4] [--off 2]
    picotool.py deploy <console-port> <file> [--dest /code.py]

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
    p = sub.add_parser("deploy"); p.add_argument("port"); p.add_argument("file")
    p.add_argument("--dest", default="/code.py"); p.set_defaults(f=cmd_deploy)
    a = ap.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
