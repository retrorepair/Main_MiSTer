import sys, wave, numpy as np
f = wave.open(sys.argv[1]); r = f.getframerate()
x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).astype(float) / 32768.0
rates = [64000, 32000, 16000, 8000, 4000, 2000, 2000, 4000, 8000, 16000, 32000, 64000]
# envelope of the 400 Hz - 20 kHz band at 20 Hz
N = 2048; hop = r // 20
w = np.hanning(N); fr = np.fft.rfftfreq(N, 1.0 / r)
band = (fr > 400) & (fr < 20000)
env = np.array([10 * np.log10((np.abs(np.fft.rfft(x[i:i + N] * w)) ** 2)[band].mean() + 1e-20) for i in range(0, len(x) - N, hop)])
env = env - np.median(env)
tt = np.arange(len(env)) / 20.0
# template: 12 bursts of 3 s every 5 s
best = None
for off in np.arange(0, 8, 0.05):
    s = 0
    for k in range(12):
        a = off + 5 * k
        m = (tt >= a + 0.3) & (tt < a + 2.7)
        g = (tt >= a + 3.3) & (tt < a + 4.7)
        if m.sum() and g.sum(): s += np.median(env[m]) - np.median(env[g])
    if best is None or s > best[0]: best = (s, off)
off = best[1]
print("burst timeline offset %.2f s (score %.1f)" % (off, best[0]))
def psd(t0, t1, NN=8192):
    seg = x[int(t0 * r):int(t1 * r)]
    ww = np.hanning(NN); acc = 0; c = 0
    for i in range(0, len(seg) - NN, NN // 2):
        acc = acc + np.abs(np.fft.rfft(seg[i:i + NN] * ww)) ** 2; c += 1
    return np.fft.rfftfreq(NN, 1.0 / r), acc / max(c, 1)
edges = [250, 500, 1000, 2000, 4000, 6000, 8000, 11000, 14000, 20000]
# floor: the gaps between bursts, medians over all gaps
floors = []
for k in range(12):
    a = off + 5 * k
    fr2, P = psd(a + 3.4, a + 4.7); floors.append(P)
F = np.median(np.array(floors), axis=0)
print("rate    " + " ".join("%5d-%-5d" % (edges[i], edges[i + 1]) for i in range(len(edges) - 1)) + "   (dB above room floor, median of the two passes)")
res = {}
for k, rt in enumerate(rates):
    a = off + 5 * k
    fr2, P = psd(a + 0.5, a + 2.8)
    row = []
    for i in range(len(edges) - 1):
        m = (fr2 >= edges[i]) & (fr2 < edges[i + 1])
        row.append(10 * np.log10(P[m].mean() / F[m].mean()))
    res.setdefault(rt, []).append(row)
for rt in sorted(res):
    row = np.median(np.array(res[rt]), axis=0)
    print("%-7d " % rt + " ".join("%11.1f" % v for v in row))
