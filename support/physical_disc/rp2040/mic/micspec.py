import sys, wave, numpy as np
from PIL import Image
f = wave.open(sys.argv[1]); rate = f.getframerate()
x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).astype(float) / 32768.0
fmax = float(sys.argv[3]) if len(sys.argv) > 3 else 8000.0
N = 2048; hop = 512
w = np.hanning(N)
cols = []
for i in range(0, len(x) - N, hop):
    S = np.abs(np.fft.rfft(x[i:i + N] * w))
    cols.append(20 * np.log10(S + 1e-7))
M = np.array(cols).T
freqs = np.fft.rfftfreq(N, 1.0 / rate)
M = M[freqs <= fmax]
lo, hi = np.percentile(M, 5), np.percentile(M, 99.7)
img = np.clip((M - lo) / (hi - lo), 0, 1)
img = (255 * (1 - img)).astype(np.uint8)[::-1]
im = Image.fromarray(img).resize((min(1600, img.shape[1]), 500))
im.save(sys.argv[2])
print("saved", sys.argv[2], "t=0..%.1fs f=0..%d Hz" % (len(x) / rate, fmax))
