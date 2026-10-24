"""v3 mix: the YC intake film's score (looped, ducked) + Chatterbox VO + synthesized sound design (sfx.py), -16 LUFS.
-> Craigie_Bridge_Inspector.mp4 (+ _share.mp4)
"""
import os, json, subprocess, numpy as np, soundfile as sf, pyloudnorm as pyln
from scipy.signal import resample_poly
H = os.path.dirname(os.path.abspath(__file__)); P = lambda *a: os.path.join(H, *a)
SR = 48000
TL = json.load(open(P("build3", "timeline.json"))); TOT = TL["total"]; N = int(TOT * SR)
ST = dict((k, s) for k, s in TL["starts"]); ORDER = [k for k, _ in TL["starts"]]
END = {k: (ORDER and (dict(zip(ORDER, [s for _, s in TL["starts"][1:]] + [TOT]))[k])) for k in ORDER}

mus, sr = sf.read(os.path.expanduser("~/biofouling-yc/video/build/music_only.wav")); assert sr == SR
XF = int(2.0 * SR); LOOP_FROM = int(28 * SR)
score = mus[: len(mus) - int(6 * SR)].copy(); outro = mus[len(mus) - int(6 * SR):]
f = np.linspace(0, 1, XF)[:, None]
while len(score) < N - len(outro) + XF:
    nxt = mus[LOOP_FROM: len(mus) - int(6 * SR)]; score = np.concatenate([score[:-XF], score[-XF:] * (1 - f) + nxt[:XF] * f, nxt[XF:]])
body = score[: N - len(outro) + XF]
score = np.concatenate([body[:-XF], body[-XF:] * (1 - f) + outro[:XF] * f, outro[XF:]])[:N]
score = np.pad(score, ((0, N - len(score)), (0, 0))); score[-int(3 * SR):] *= np.linspace(1, 0, int(3 * SR))[:, None] ** 2

def wav(n):
    x, s = sf.read(P("sfx", f"{n}.wav")); return x if s == SR else resample_poly(x, SR, s, axis=0)
fx = np.zeros((N, 2))
def bed(name, keys, gain):
    src = wav(name)
    for k in keys:
        if k not in ST: continue
        a, b = int(ST[k] * SR), int(min(END[k] + 0.5, TOT) * SR); L = b - a
        x = np.tile(src, (L // len(src) + 1, 1))[:L].copy(); r = min(int(0.4 * SR), L // 3)
        x[:r] *= np.linspace(0, 1, r)[:, None]; x[-r:] *= np.linspace(1, 0, r)[:, None]; fx[a:b] += x * gain
def hit(name, t, gain):
    x = wav(name); i = int(max(t, 0) * SR); L = min(len(x), N - i); fx[i:i + L] += x[:L] * gain
bed("drone_hum", ["o1", "o2", "title", "b1", "x1", "x2", "l1", "c2"], 0.30)
bed("drone_hum", ["m1", "m2", "x3"], 0.18)
bed("underwater", ["o3", "p1", "p3", "p4", "p5", "n1", "n2", "n3", "x4", "x5"], 0.55)
for k in ("n3", "x3", "x5"): hit("sonar_ping", ST[k] + 0.6, 0.35)
for k in ("p1", "q1", "b1", "m1", "n1", "x1", "k1", "v1", "l1"): hit("whoosh", ST[k] - 0.6, 0.45)
hit("impact", TL["title"][0] - 0.05, 0.9)

voice = np.zeros(N); duck = np.zeros(N)
for k, t, d in TL["vo"]:
    x, vsr = sf.read(P("vo3", f"{k}.wav")); x = x.mean(1) if x.ndim > 1 else x
    x = resample_poly(x, SR, vsr); i = int(t * SR); voice[i:i + len(x)] += x[: N - i]; duck[max(0, i - int(0.25 * SR)): i + len(x) + int(0.35 * SR)] = 1
k = int(0.5 * SR); duck = np.convolve(duck, np.ones(k) / k, mode="same")
meter = pyln.Meter(SR)
voice *= 10 ** ((-16.5 - meter.integrated_loudness(voice)) / 20)
score *= 10 ** ((-22.0 - meter.integrated_loudness(score)) / 20)
fx *= 10 ** ((-27.0 - meter.integrated_loudness(fx)) / 20)
mix = score * (1.0 - 0.68 * duck)[:, None] + fx * (1.0 - 0.6 * duck)[:, None] + voice[:, None]   # voice clearly on top
mix *= 10 ** ((-16.0 - meter.integrated_loudness(mix)) / 20)
pk = np.abs(mix).max()
if pk > 0.97: mix *= 0.97 / pk
sf.write(P("build3", "mix.wav"), mix.astype(np.float32), SR)
print(f"mix {N / SR:.1f}s {meter.integrated_loudness(mix):.1f} LUFS")
out = P("Craigie_Bridge_Inspector.mp4")
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", P("build3", "video_noaudio.mp4"), "-i", P("build3", "mix.wav"), "-map", "0:v", "-map", "1:a",
                "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-shortest", "-movflags", "+faststart", out], check=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", out, "-c:v", "libx264", "-preset", "slow", "-crf", "22", "-maxrate", "8M", "-bufsize", "16M",
                "-c:a", "copy", "-movflags", "+faststart", P("Craigie_Bridge_Inspector_share.mp4")], check=True)
print("DONE", out)
