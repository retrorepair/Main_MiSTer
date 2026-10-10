import sys, wave, numpy as np
f = wave.open(sys.argv[1]); r = f.getframerate()
x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).astype(float) / 32768.0
N = 2048; hop = 480
w = np.hanning(N); fr = np.fft.rfftfreq(N, 1.0 / r)
hi = (fr > 6000) & (fr < 16000)
env = np.array([10 * np.log10((np.abs(np.fft.rfft(x[i:i + N] * w)) ** 2)[hi].mean() + 1e-20) for i in range(0, len(x) - N, hop)])
tt = np.arange(len(env)) * hop / r
k = np.where(env > np.median(env) + 12)[0]
tm = tt[k[0]]
print("marker at %.2f s" % tm)
NN = 16384; ww = np.hanning(NN); fq = np.fft.rfftfreq(NN, 1.0 / r)
def spec(t0, t1):
    seg = x[int((tm + t0) * r):int((tm + t1) * r)]
    acc = 0; c = 0
    for i in range(0, len(seg) - NN, NN // 2):
        acc = acc + np.abs(np.fft.rfft(seg[i:i + NN] * ww)) ** 2; c += 1
    return acc / max(c, 1)
def lines(S, f0):
    out = []
    for h in (1, 2, 3, 4, 5):
        fc = f0 * h
        k = (fq > fc - 4) & (fq < fc + 4); sd = ((fq > fc - 60) & (fq < fc - 20)) | ((fq > fc + 20) & (fq < fc + 60))
        out.append(10 * np.log10(S[k].max() + 1e-20) - 10 * np.log10(np.median(S[sd]) + 1e-20))
    return out
def band(S, lo, hi_):
    m = (fq >= lo) & (fq < hi_); return 10 * np.log10(S[m].mean() + 1e-20)
quiet = spec(0.6, 1.4)
for name, a, b in [("quiet gap before", 0.6, 1.4), ("RIN 300 Hz (inward)", 1.9, 4.5), ("quiet gap", 5.0, 6.6), ("FIN 300 Hz (outward)", 6.9, 9.4)]:
    S = spec(a, b)
    ln = lines(S, 300)
    print("%-22s 300/600/900/1200/1500 Hz lines over neighbours: %s | 200-2000 Hz band %5.1f dB (quiet gap %5.1f)" % (name, " ".join("%5.1f" % v for v in ln), band(S, 200, 2000), band(quiet, 200, 2000)))
