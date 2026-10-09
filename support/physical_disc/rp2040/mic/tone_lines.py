import sys, wave, numpy as np
f = wave.open(sys.argv[1]); r = f.getframerate()
x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).astype(float) / 32768.0
labels = sys.argv[2].split(",")
N = 8192; hop = 1024
w = np.hanning(N); fr = np.fft.rfftfreq(N, 1.0 / r)
def line(S, fc):
    k = (fr > fc - 15) & (fr < fc + 15); sd = ((fr > fc - 150) & (fr < fc - 50)) | ((fr > fc + 50) & (fr < fc + 150))
    return 10 * np.log10(S[k].max() + 1e-20) - 10 * np.log10(np.median(S[sd]) + 1e-20)
ts = []; L = {1000: [], 3000: [], 5000: []}
for i in range(0, len(x) - N, hop):
    S = np.abs(np.fft.rfft(x[i:i + N] * w)) ** 2
    ts.append(i / r)
    for fc in L: L[fc].append(line(S, fc))
ts = np.array(ts); L = {k: np.array(v) for k, v in L.items()}
score_sig = L[1000]
best = None
n = len(labels)
for per in np.arange(2.95, 3.35, 0.01):
    for off in np.arange(0, 8, 0.05):
        s = 0
        for k in range(n):
            a = off + per * k
            m = (ts >= a + 0.1) & (ts < a + 0.4); g = (ts >= a + 1.2) & (ts < a + 2.4)
            if m.sum() and g.sum(): s += np.median(score_sig[m]) - np.median(score_sig[g])
        if best is None or s > best[0]: best = (s, off, per)
s, off, per = best
print("alignment offset %.2f s period %.2f s (score %.1f)" % (off, per, s))
print("tone     1k line   3k line   5k line   (dB above the neighbouring spectrum, median over the 0.5 s tone)")
for k, lab in enumerate(labels):
    a = off + per * k
    m = (ts >= a + 0.1) & (ts < a + 0.4)
    print("%-8s %7.1f  %7.1f  %7.1f" % (lab, np.median(L[1000][m]), np.median(L[3000][m]), np.median(L[5000][m])))
