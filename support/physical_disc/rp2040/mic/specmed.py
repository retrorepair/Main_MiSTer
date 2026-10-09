import sys, wave, numpy as np
def load(p):
    f = wave.open(p); r = f.getframerate(); n = f.getnchannels()
    x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).astype(float) / 32768.0
    if n > 1: x = x.reshape(-1, n).mean(1)
    return x, r
def med(x, r, t0, t1, q=50, N=4096):
    seg = x[int(t0 * r):int(t1 * r)]
    w = np.hanning(N); fr = np.fft.rfftfreq(N, 1.0 / r); rows = []
    for i in range(0, len(seg) - N, N // 2):
        S = np.abs(np.fft.rfft(seg[i:i + N] * w)) ** 2
        rows.append(S)
    rows = np.array(rows)
    edges = [250, 500, 1000, 2000, 3000, 4000, 6000, 8000, 11000, 14000]
    out = []
    for i in range(len(edges) - 1):
        k = (fr >= edges[i]) & (fr < edges[i + 1])
        band = 10 * np.log10(rows[:, k].mean(1) + 1e-20)
        out.append(np.percentile(band, q))
    return edges, out
a = sys.argv
xa, ra = load(a[1]); xb, rb = load(a[4])
e, A = med(xa, ra, float(a[2]), float(a[3]), float(a[7]) if len(a) > 7 else 50)
e, B = med(xb, rb, float(a[5]), float(a[6]), 50)
print("band(Hz)     A(dB)  floor(dB)  A-floor")
for i in range(len(A)):
    print("%5d-%-6d %7.1f %9.1f %8.1f" % (e[i], e[i + 1], A[i], B[i], A[i] - B[i]))
