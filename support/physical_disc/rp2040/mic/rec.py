"""Record the default Windows microphone to a WAV with winmm via ctypes (no packages needed).

    python rec.py out.wav seconds [rate]
"""
import ctypes, sys, time, wave
from ctypes import wintypes as w

winmm = ctypes.WinDLL("winmm")


class WAVEFORMATEX(ctypes.Structure):
    _fields_ = [("wFormatTag", w.WORD), ("nChannels", w.WORD), ("nSamplesPerSec", w.DWORD),
                ("nAvgBytesPerSec", w.DWORD), ("nBlockAlign", w.WORD), ("wBitsPerSample", w.WORD),
                ("cbSize", w.WORD)]


class WAVEHDR(ctypes.Structure):
    pass


WAVEHDR._fields_ = [("lpData", ctypes.c_void_p), ("dwBufferLength", w.DWORD), ("dwBytesRecorded", w.DWORD),
                    ("dwUser", ctypes.c_void_p), ("dwFlags", w.DWORD), ("dwLoops", w.DWORD),
                    ("lpNext", ctypes.c_void_p), ("reserved", ctypes.c_void_p)]

winmm.waveInOpen.argtypes = [ctypes.POINTER(ctypes.c_void_p), w.UINT, ctypes.POINTER(WAVEFORMATEX),
                             ctypes.c_void_p, ctypes.c_void_p, w.DWORD]
for name in ("waveInPrepareHeader", "waveInAddBuffer", "waveInUnprepareHeader"):
    getattr(winmm, name).argtypes = [ctypes.c_void_p, ctypes.POINTER(WAVEHDR), w.UINT]
for name in ("waveInStart", "waveInStop", "waveInReset", "waveInClose"):
    getattr(winmm, name).argtypes = [ctypes.c_void_p]

out = sys.argv[1]
secs = float(sys.argv[2])
rate = int(sys.argv[3]) if len(sys.argv) > 3 else 44100

fmt = WAVEFORMATEX(1, 1, rate, rate * 2, 2, 16, 0)
h = ctypes.c_void_p()
r = winmm.waveInOpen(ctypes.byref(h), 0xFFFFFFFF, ctypes.byref(fmt), None, None, 0)
if r:
    sys.exit("waveInOpen failed %d" % r)

chunk = rate // 4                                   # quarter-second buffers
n = int(secs * 4) + 2
bufs = [ctypes.create_string_buffer(chunk * 2) for _ in range(n)]
hdrs = []
for b in bufs:
    hd = WAVEHDR(ctypes.cast(b, ctypes.c_void_p), chunk * 2, 0, None, 0, 0, None, None)
    winmm.waveInPrepareHeader(h, ctypes.byref(hd), ctypes.sizeof(hd))
    winmm.waveInAddBuffer(h, ctypes.byref(hd), ctypes.sizeof(hd))
    hdrs.append(hd)
winmm.waveInStart(h)
print("recording %.1f s" % secs, flush=True)
t0 = time.time()
time.sleep(secs)
winmm.waveInStop(h)
winmm.waveInReset(h)
data = b""
for b, hd in zip(bufs, hdrs):
    data += b.raw[:hd.dwBytesRecorded]
    winmm.waveInUnprepareHeader(h, ctypes.byref(hd), ctypes.sizeof(hd))
winmm.waveInClose(h)
with wave.open(out, "wb") as f:
    f.setnchannels(1)
    f.setsampwidth(2)
    f.setframerate(rate)
    f.writeframes(data)
print("wrote %s: %d samples (%.2f s)" % (out, len(data) // 2, len(data) / 2.0 / rate), flush=True)
