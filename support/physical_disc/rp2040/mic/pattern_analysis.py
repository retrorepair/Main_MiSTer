import sys, wave, numpy as np
f = wave.open(sys.argv[1]); r = f.getframerate()
x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).astype(float) / 32768.0
ev = []
for l in open(sys.argv[2]):
    p = l.split()
    if len(p) == 2 and p[0] != "END":
        try: ev.append((p[0], float(p[1])))
        except ValueError: pass
N = 1024; hop = 480
w = np.hanning(N); fr = np.fft.rfftfreq(N, 1.0 / r)
hi = (fr > 6000) & (fr < 16000)
env = np.array([10 * np.log10((np.abs(np.fft.rfft(x[i:i + N] * w)) ** 2)[hi].mean() + 1e-20) for i in range(0, len(x) - N, hop)])
tt = np.arange(len(env)) * hop / r
base = np.median(env)
k = np.where(env > base + 15)[0]
t_mark = tt[k[0]]
print("marker found at %.2f s in the recording (%.0f dB over median)" % (t_mark, env[k[0]] - base))
def bandpow(t0, t1, lo, hi_):
    seg = x[int((t_mark + t0) * r):int((t_mark + t1) * r)]
    if len(seg) < 2048: return np.nan
    NN = 2048; ww = np.hanning(NN); fq = np.fft.rfftfreq(NN, 1.0 / r); acc = 0; c = 0
    for i in range(0, len(seg) - NN, NN // 2):
        acc = acc + np.abs(np.fft.rfft(seg[i:i + NN] * ww)) ** 2; c += 1
    m = (fq >= lo) & (fq < hi_)
    return 10 * np.log10((acc / max(c, 1))[m].mean() + 1e-20)
bands = [(60, 200), (200, 600), (600, 2000), (2000, 6000), (6000, 16000)]
def report(name_on, name_off, on_win, off_win):
    ons = [t for n, t in ev if n == name_on]; offs = [t for n, t in ev if n == name_off]
    print("\n%s (window %.2f-%.2f after the command) minus %s (window %.2f-%.2f)   n=%d" % (name_on, on_win[0], on_win[1], name_off, off_win[0], off_win[1], min(len(ons), len(offs))))
    print("band(Hz)       mean diff dB   se   z")
    for lo, hi_ in bands:
        d = []
        for a, b in zip(ons, offs):
            d.append(bandpow(a + on_win[0], a + on_win[1], lo, hi_) - bandpow(b + off_win[0], b + off_win[1], lo, hi_))
        d = np.array([v for v in d if not np.isnan(v)])
        se = d.std(ddof=1) / np.sqrt(len(d)) if len(d) > 1 else float("nan")
        print("%5d-%-6d %9.2f %7.2f %6.1f" % (lo, hi_, d.mean(), se, d.mean() / se if se else float("nan")))
report("SPIN_ON", "SPIN_OFF", (0.2, 0.95), (0.2, 0.95))
report("SLED_OUT", "SLED_IN", (0.0, 0.3), (0.0, 0.3))
# sled pulses vs the quiet gap after them
ons = [t for n, t in ev if n in ("SLED_OUT", "SLED_IN")]
print("\nSLED pulses (either direction, 0-0.3 s after the command) minus the quiet gap (0.7-1.1 s), n=%d" % len(ons))
for lo, hi_ in bands:
    d = np.array([bandpow(a, a + 0.3, lo, hi_) - bandpow(a + 0.7, a + 1.1, lo, hi_) for a in ons])
    se = d.std(ddof=1) / np.sqrt(len(d))
    print("%5d-%-6d %9.2f %7.2f %6.1f" % (lo, hi_, d.mean(), se, d.mean() / se))
