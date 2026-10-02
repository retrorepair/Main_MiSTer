#!/usr/bin/env python3
"""Is this drive any good for acoustic mirroring?

Characterises any optical drive against what a Mega CD actually does, so a
candidate can be judged in a minute instead of by ear over an evening. Needs an
audio or mixed-mode CD in the drive (a full one, so the whole stroke is usable).

  python3 drivecheck.py [/dev/sr0]

The number that matters is full-stroke velocity. A Mega CD's sled runs 16-21 mm/s;
anything near that reads as a console, anything near 50 reads as a modern drive.
"""
import ctypes, fcntl, math, os, struct, sys, time

SG_IO = 0x2285
dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/sr0"
R_IN, R_OUT = 24.0, 58.0

fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)

def sgio(cdb, dxfer_len=0, direction=-1, timeout=20000):
    cmd   = ctypes.create_string_buffer(bytes(cdb), len(cdb))
    sense = ctypes.create_string_buffer(32)
    data  = ctypes.create_string_buffer(max(dxfer_len, 1))
    hdr = struct.pack("iiBBHIPPPIIiPBBBBHHiII",
                      ord('S'), direction, len(cdb), 32, 0, dxfer_len,
                      ctypes.addressof(data) if dxfer_len else 0,
                      ctypes.addressof(cmd), ctypes.addressof(sense),
                      timeout, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    buf = ctypes.create_string_buffer(hdr, len(hdr))
    fcntl.ioctl(fd, SG_IO, buf)
    o = struct.unpack("iiBBHIPPPIIiPBBBBHHiII", bytes(buf))
    return (o[13] | o[17] | o[18]), data.raw[:dxfer_len]

def toc_leadout():
    err, d = sgio([0x43, 0x00, 0x00, 0,0,0, 0, 0x03, 0x24, 0], 0x324, -3)
    if err: return None
    ln = struct.unpack(">H", d[0:2])[0]
    for i in range((ln - 2) // 8):
        e = d[4+i*8 : 12+i*8]
        if len(e) >= 8 and e[2] == 0xAA:
            return struct.unpack(">i", e[4:8])[0]
    return None

FULL = toc_leadout()
if not FULL or FULL < 100000:
    print("need a reasonably full CD in the drive (leadout %s)" % FULL); sys.exit(1)
print("disc leadout %d sectors (%.0f%% of a full CD)\n" % (FULL, 100.0*FULL/333000))

def radius(lba):
    f = min(max(lba/float(FULL), 0.0), 1.0)
    return math.sqrt(R_IN*R_IN + f*(R_OUT*R_OUT - R_IN*R_IN))
def lba_at(r):
    f = (r*r - R_IN*R_IN)/(R_OUT*R_OUT - R_IN*R_IN)
    return int(max(0.0, min(1.0, f))*FULL)

def seek(l): return sgio([0x2B,0,(l>>24)&255,(l>>16)&255,(l>>8)&255,l&255,0,0,0,0])[0]
def play(l, b=20000):
    return sgio([0x45,0,(l>>24)&255,(l>>16)&255,(l>>8)&255,l&255,0,(b>>8)&255,b&255,0])[0]
def subq():
    e,d = sgio([0x42,0x00,0x40,0x01,0,0,0,0,16,0], 16, -3)
    return None if (e or len(d)<16) else struct.unpack(">i", d[8:12])[0]
def synced(l):
    t = time.time(); seek(l); subq(); return (time.time()-t)*1000.0

LO, HI = lba_at(R_IN+1.0), lba_at(R_OUT-1.0)
STROKE = radius(HI) - radius(LO)

print("1. full-stroke seek (%.1f mm), median of 5" % STROKE)
v = []
for _ in range(5):
    play(LO); time.sleep(0.4); subq()
    v.append(synced(HI))
ms = sorted(v)[2]
vel = STROKE*1000.0/ms
print("   %.0f ms  ->  %.0f mm/s" % (ms, vel))

print("\n2. back-to-back leg cost (what segmentation would cost)")
for L in (4.0, 8.0):
    play(LO); time.sleep(0.4)
    r = radius(LO); t0 = time.time(); n = 0
    while r + L < radius(HI) and n < 6:
        r += L; seek(lba_at(r)); n += 1
    dt = (time.time()-t0)*1000.0
    if n:
        print("   %.0f mm legs: %.0f ms each -> %.0f mm/s" % (L, dt/n, L*1000.0/(dt/n)))

print("\n3. useful primitives")
print("   SCAN (0xBA)      : %s" % ("accepted" if not sgio([0xBA,0,0,0,0x10,0,0,0,0,0,0,0]) else "rejected"))
e1 = sgio([0xBB,0,0,176,0xFF,0xFF,0,0,0,0,0,0])[0]
play(LO); time.sleep(0.3); slow = synced(HI)
sgio([0xBB,0,0xFF,0xFF,0xFF,0xFF,0,0,0,0,0,0])
play(LO); time.sleep(0.3); fast = synced(HI)
print("   SET CD SPEED     : %s (1x %.0f ms vs max %.0f ms, %.0f%% difference)"
      % ("honoured" if not e1 else "rejected", slow, fast,
         100.0*(slow-fast)/fast if fast else 0))

sgio([0x4E,0,0,0,0,0]); os.close(fd)

print("\n=== verdict ===")
print("   a Mega CD sled is 16-21 mm/s, full stroke 1.5-2 s")
if vel <= 24:
    print("   %.0f mm/s: EXCELLENT. One continuous sweep per seek, no segmentation." % vel)
elif vel <= 32:
    print("   %.0f mm/s: GOOD. Two or three legs per seek." % vel)
elif vel <= 45:
    print("   %.0f mm/s: USABLE. Four or more legs, so audible stop/start impulses." % vel)
else:
    print("   %.0f mm/s: TOO FAST. Needs heavy segmentation and will clatter." % vel)
