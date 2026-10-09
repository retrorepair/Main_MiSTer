"""On the MiSTer: drive the spindle, then the sled, in a fixed on/off pattern and print timestamps relative to a
loud lens-noise marker, so the PC microphone recording can be checked for sound that follows the pattern."""
import os, select, sys, time, termios

PORT = "/dev/ttyACM1"


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    t = termios.tcgetattr(fd)
    t[0] = 0; t[1] = 0; t[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; t[3] = 0
    t[4] = t[5] = termios.B115200
    t[6][termios.VMIN] = 0; t[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, t)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


fd = open_raw(PORT)


def tx(line):
    os.write(fd, (line + "\n").encode())


def drain(sec):
    end = time.time() + sec
    out = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                out += os.read(fd, 4096)
            except BlockingIOError:
                pass
    return out.decode("ascii", "replace")


tx("STOP"); drain(0.3)
t0 = time.time()
tx("LENS B Z 3 300 8000")
print("MARKER 0.000", flush=True)
drain(1.5)
cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 10
which = sys.argv[2] if len(sys.argv) > 2 else "both"
if which in ("spin", "both"):
    for i in range(cycles):
        tx("SPIN 600 50"); print("SPIN_ON %.3f" % (time.time() - t0), flush=True)
        drain(1.0)
        tx("SPIN 0 50"); print("SPIN_OFF %.3f" % (time.time() - t0), flush=True)
        drain(1.0)
    drain(1.0)
if which in ("sled", "both"):
    for i in range(cycles):
        tx("DRIVE out 0.9 300"); print("SLED_OUT %.3f" % (time.time() - t0), flush=True)
        drain(1.2)
        tx("DRIVE in 0.9 300"); print("SLED_IN %.3f" % (time.time() - t0), flush=True)
        drain(1.2)
tx("STOP"); drain(0.3)
print("END %.3f" % (time.time() - t0), flush=True)
