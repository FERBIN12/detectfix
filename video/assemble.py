"""Mix (YC intake score, ducked under the Chatterbox VO, -16 LUFS) + mux -> Above_and_Below_3min.mp4 (+ _share.mp4)."""
import os, json, subprocess, numpy as np, soundfile as sf, pyloudnorm as pyln
from scipy.signal import resample_poly
H = os.path.dirname(os.path.abspath(__file__)); P = lambda *a: os.path.join(H, *a)
SR = 48000
TL = json.load(open(P("build", "timeline.json"))); TOT = TL["total"]; N = int(TOT * SR)
mus, sr = sf.read(os.path.expanduser("~/biofouling-yc/video/build/music_only.wav")); assert sr == SR
XF = int(2.0 * SR); LOOP_FROM = int(28 * SR)
score = mus[: len(mus) - int(6 * SR)].copy(); outro = mus[len(mus) - int(6 * SR):]
f = np.linspace(0, 1, XF)[:, None]
while len(score) < N - len(outro) + XF:
    nxt = mus[LOOP_FROM: len(mus) - int(6 * SR)]
    score = np.concatenate([score[:-XF], score[-XF:] * (1 - f) + nxt[:XF] * f, nxt[XF:]])
body = score[: N - len(outro) + XF]
score = np.concatenate([body[:-XF], body[-XF:] * (1 - f) + outro[:XF] * f, outro[XF:]])[:N]
score = np.pad(score, ((0, N - len(score)), (0, 0)))
score[-int(3 * SR):] *= np.linspace(1, 0, int(3 * SR))[:, None] ** 2
voice = np.zeros(N); duck = np.zeros(N)
for k, t, d in TL["vo"]:
    x, vsr = sf.read(P("vo", f"{k}.wav"))
    if x.ndim > 1: x = x.mean(1)
    x = resample_poly(x, SR, vsr); i = int(t * SR); voice[i:i + len(x)] += x[: N - i]
    duck[max(0, i - int(0.3 * SR)): i + len(x) + int(0.4 * SR)] = 1
k = int(0.35 * SR); duck = np.convolve(duck, np.ones(k) / k, mode="same")
meter = pyln.Meter(SR)
voice *= 10 ** ((-18.0 - meter.integrated_loudness(voice)) / 20)
score *= 10 ** ((-23.0 - meter.integrated_loudness(score)) / 20)
mix = score * (1.0 - 0.55 * duck)[:, None] + voice[:, None]
mix *= 10 ** ((-16.0 - meter.integrated_loudness(mix)) / 20)
pk = np.abs(mix).max()
if pk > 0.95: mix *= 0.95 / pk
sf.write(P("build", "mix.wav"), mix.astype(np.float32), SR)
print(f"mix {N / SR:.1f}s  {meter.integrated_loudness(mix):.1f} LUFS")
out = P("Above_and_Below_3min.mp4")
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", P("build", "video_noaudio.mp4"), "-i", P("build", "mix.wav"), "-map", "0:v", "-map", "1:a",
                "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-shortest", out], check=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", out, "-c:v", "libx264", "-preset", "medium", "-crf", "26", "-maxrate", "4M", "-bufsize", "8M",
                "-c:a", "copy", P("Above_and_Below_3min_share.mp4")], check=True)
print("DONE", out)
