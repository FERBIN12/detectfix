"""Music + VO mix for the v2 cut and mux -> Above_and_Below.mp4 (+ _share.mp4).
Score: Kevin MacLeod (incompetech.com, CC BY 4.0): "Lightless Dawn" under the pain act, cross-fading into "Impact Prelude"
at the insight so its build peaks over money / why-now and its own ending lands on the end card; "Discovery Hit" on the
title reveal and the 3D-model reveal. Ducked under the Chatterbox VO; -16 LUFS.
"""
import os, json, subprocess, numpy as np, soundfile as sf, pyloudnorm as pyln, librosa
from scipy.signal import resample_poly
H = os.path.dirname(os.path.abspath(__file__)); P = lambda *a: os.path.join(H, *a)
SR = 48000
TL = json.load(open(P("build2", "timeline.json"))); TOT = TL["total"]; N = int(TOT * SR)
VT = {k: (t, d) for k, t, d in TL["vo"]}


def load(name):
    y, _ = librosa.load(P("music", name), sr=SR, mono=False)
    return (np.stack([y, y]) if y.ndim == 1 else y).T


def fade(x, a, b):
    x = x.copy(); na, nb = int(a * SR), int(b * SR)
    if na: x[:na] *= np.linspace(0, 1, na)[:, None]
    if nb: x[-nb:] *= np.linspace(1, 0, nb)[:, None]
    return x


score = np.zeros((N, 2))
T_INS = VT["i1"][0] - 1.2                                       # the turn: dark -> rising
dark = load("Lightless_Dawn.mp3")[int(12 * SR):]
dark = fade(dark[: int((T_INS + 2.5) * SR)], 2.5, 2.5); score[: len(dark)] += dark * 1.15
rise = load("Impact_Prelude.mp3"); rise = rise[: int((len(rise) / SR - 2.0) * SR)]   # drop the tail silence
need = N - int(T_INS * SR); off = max(0, len(rise) - need)
rise = fade(rise[off:], 3.0, 0.0); i0 = int(T_INS * SR); score[i0: i0 + len(rise)] += rise[: N - i0]
hit = load("Discovery_Hit.mp3")
for t in (TL["title"][0] - 0.15, VT["t3"][0] - 0.4):
    i = int(t * SR); h = fade(hit[: N - i], 0.0, 3.0); score[i: i + len(h)] += h * 0.8
score[-int(4 * SR):] *= np.linspace(1, 0, int(4 * SR))[:, None] ** 1.5

voice = np.zeros(N); duck = np.zeros(N)
for k, (t, d) in VT.items():
    x, vsr = sf.read(P("vo2", f"{k}.wav"))
    if x.ndim > 1: x = x.mean(1)
    x = resample_poly(x, SR, vsr); i = int(t * SR); voice[i:i + len(x)] += x[: N - i]
    duck[max(0, i - int(0.25 * SR)): i + len(x) + int(0.35 * SR)] = 1
k = int(0.5 * SR); duck = np.convolve(duck, np.ones(k) / k, mode="same")
meter = pyln.Meter(SR)
voice *= 10 ** ((-17.5 - meter.integrated_loudness(voice)) / 20)
score *= 10 ** ((-21.0 - meter.integrated_loudness(score)) / 20)
mix = score * (1.0 - 0.62 * duck)[:, None] + voice[:, None]
mix *= 10 ** ((-16.0 - meter.integrated_loudness(mix)) / 20)
pk = np.abs(mix).max()
if pk > 0.97: mix *= 0.97 / pk
sf.write(P("build2", "mix.wav"), mix.astype(np.float32), SR)
print(f"mix {N / SR:.1f}s {meter.integrated_loudness(mix):.1f} LUFS, turn at {T_INS:.1f}s, Impact Prelude from {off / SR:.1f}s")
out = P("Above_and_Below.mp4")
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", P("build2", "video_noaudio.mp4"), "-i", P("build2", "mix.wav"), "-map", "0:v", "-map", "1:a",
                "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-shortest", "-movflags", "+faststart", out], check=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", out, "-c:v", "libx264", "-preset", "slow", "-crf", "23", "-maxrate", "6M", "-bufsize", "12M",
                "-c:a", "copy", "-movflags", "+faststart", P("Above_and_Below_share.mp4")], check=True)
print("DONE", out)
