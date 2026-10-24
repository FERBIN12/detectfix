"""Synthesized sound design (48 kHz stereo wav, royalty-free by construction) -> sfx/*.wav
drone_hum (multi-rotor blade-pass + motor whine), underwater (muffled rumble + bubbles), sonar_ping, whoosh, impact.
"""
import os, numpy as np, soundfile as sf
from scipy.signal import butter, sosfilt
SR = 48000; H = os.path.dirname(os.path.abspath(__file__)); os.makedirs(os.path.join(H, "sfx"), exist_ok=True)
rng = np.random.default_rng(7)
def lp(x, f, o=4): return sosfilt(butter(o, f / (SR / 2), "low", output="sos"), x, axis=0)
def hp(x, f, o=4): return sosfilt(butter(o, f / (SR / 2), "high", output="sos"), x, axis=0)
def bp(x, a, b, o=4): return sosfilt(butter(o, [a / (SR / 2), b / (SR / 2)], "band", output="sos"), x, axis=0)
def st(x, w=0.15):
    d = int(0.011 * SR); y = np.stack([x, np.roll(x, d)], 1); return y * (1 - w) + y[:, ::-1] * w
def norm(x, pk=0.8): return x / (np.abs(x).max() + 1e-9) * pk

# drone: four rotors near 170-190 Hz blade-pass with slow wander, harmonics, plus broadband air noise
T = 20.0; t = np.arange(int(T * SR)) / SR; x = np.zeros_like(t)
for k, f0 in enumerate((172.0, 179.0, 185.0, 191.0)):
    f = f0 * (1 + 0.012 * np.sin(2 * np.pi * (0.13 + 0.05 * k) * t + k)); ph = 2 * np.pi * np.cumsum(f) / SR
    for h, a in ((1, 1.0), (2, 0.45), (3, 0.25), (4, 0.12)): x += a * np.sin(h * ph + k)
x = x * 0.25 + bp(rng.standard_normal(len(t)), 300, 4000) * 0.35
sf.write(os.path.join(H, "sfx", "drone_hum.wav"), st(norm(lp(x, 6000)), 0.1), SR)

# underwater: deep rumble, pressure hiss, and occasional bubble trains
x = lp(rng.standard_normal(len(t)), 220, 4) * 3.0 + lp(hp(rng.standard_normal(len(t)), 300), 1200) * 0.15
for _ in range(14):
    t0 = rng.uniform(0, T - 1); n = rng.integers(3, 9)
    for j in range(n):
        tb = t0 + j * rng.uniform(0.04, 0.12); f = rng.uniform(500, 1400); d = int(0.05 * SR); i = int(tb * SR)
        if i + d >= len(x): continue
        tt = np.arange(d) / SR; x[i:i + d] += 0.25 * np.sin(2 * np.pi * f * tt * (1 + 3 * tt)) * np.exp(-tt * 60)
sf.write(os.path.join(H, "sfx", "underwater.wav"), st(norm(lp(x, 2500)), 0.3), SR)

# sonar ping: clean sine with a long watery tail
tt = np.arange(int(2.5 * SR)) / SR
x = np.sin(2 * np.pi * 1850 * tt) * np.exp(-tt * 3.0) * (1 - np.exp(-tt * 400))
x += 0.3 * np.sin(2 * np.pi * 1850 * (tt - 0.35).clip(0)) * np.exp(-(tt - 0.35).clip(0) * 4) * (tt > 0.35)
sf.write(os.path.join(H, "sfx", "sonar_ping.wav"), st(norm(x, 0.6), 0.25), SR)

# whoosh: noise through a sweeping band-pass, swelling then fading
d = int(1.2 * SR); tt = np.arange(d) / SR; n = rng.standard_normal(d); y = np.zeros(d); blk = 1024
for i in range(0, d, blk):
    c = 300 + 3500 * (i / d) ** 1.5; y[i:i + blk] = bp(n[i:i + blk], c * 0.6, min(c * 1.6, 20000))
env = np.sin(np.pi * tt / tt[-1]) ** 2
sf.write(os.path.join(H, "sfx", "whoosh.wav"), st(norm(y * env, 0.7), 0.4), SR)

# impact: sub drop + noise burst for the title
d = int(3.0 * SR); tt = np.arange(d) / SR
x = np.sin(2 * np.pi * (55 * np.exp(-tt * 0.8)) * tt) * np.exp(-tt * 1.4) + lp(rng.standard_normal(d), 900) * np.exp(-tt * 9) * 0.8
sf.write(os.path.join(H, "sfx", "impact.wav"), st(norm(x, 0.9), 0.2), SR)
print("sfx ok")
