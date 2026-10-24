"""Story cut v3: 30 fps replay footage (sim/replay_film.py), hero shots with animated sensor callouts, split-screen handoff,
kinetic numbers over the murky ROV camera, cards reused from edit2, xfade transitions.
    ~/biofouling-yc/voice/.venv/bin/python edit3.py   -> build3/video_noaudio.mp4 + build3/timeline.json
"""
import os, json, subprocess, glob
import numpy as np, soundfile as sf
from PIL import Image, ImageDraw
H = os.path.dirname(os.path.abspath(__file__)); P = lambda *a: os.path.join(H, *a)
exec(open(P("edit2.py")).read().split('NBI = "FHWA')[0])                 # card painters, fonts, colours from the v2 cut
B3 = lambda *a: os.path.join(H, "build3", *a)
def ff(args): subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + args, check=True)
ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "17", "-pix_fmt", "yuv420p", "-r", str(FPS)]
BASE = f"scale={W}:{HH}:force_original_aspect_ratio=increase,crop={W}:{HH},setsar=1"
DASH = [P("dash", f"dash_{i}.mp4") for i in range(4)]
os.makedirs(B3("segs"), exist_ok=True); os.makedirs(B3("ov"), exist_ok=True)
RP = lambda *a: os.path.join(H, "replay", *a)
SCRIPT = json.load(open(P("script3_captions.json"))); ORDER = [k for k, _ in SCRIPT]

GAP, LEAD = 0.5, 1.8
PAUSE = {"o1": 0.6, "o2": 0.4, "o3": 1.0, "p4": 0.6, "p5": 1.4, "q1": 0.4, "q2": 1.8, "b2": 0.9, "m2": 1.0, "n3": 1.2, "x2": 0.6, "x5": 0.8, "x6": 7.0, "k3": 1.0, "v3": 1.0, "l1": 0.6}
t, TL = LEAD, []
for k in ORDER:
    d = sf.info(P("vo3", f"{k}.wav")).duration; TL.append((k, t, d)); t += d + GAP + PAUSE.get(k, 0.0)
TOTAL = t + 5.0
ACTS = {"p1", "q1", "b1", "m1", "n1", "x1", "k1", "v1", "l1"}
TITLE = (TL[2][1] + TL[2][2] + 0.4, 3.2)                                       # title card after o3

NBI = "FHWA National Bridge Inventory 2022, Massachusetts"
def kinetic(big, small, color=AMBER):
    def f(d):
        d.rectangle([0, 0, W, HH], fill=(0, 0, 0, 70))
        d.text((W // 2, 590), big, font=F(BC, 260), fill=color + (255,), anchor="ms")
        if small: d.text((W // 2, 670), small, font=F(BR, 48), fill=WHITE + (255,), anchor="ms")
    return f

DRONE_CALL = {"lidar": "LIDAR · Ouster OS0-128, 128 beams", "gimbal": "GIMBAL CAMERA + THERMAL (sim)", "flood": "LED FLOODLIGHTS", "gnss": "RTK GPS"}
ROV_CALL = {"dvl": "DVL · navigates by sound", "sonar": "MULTIBEAM SONAR · 130°, 12 m", "camera": "LOW-LIGHT CAMERA", "lumen": "LUMEN LIGHTS"}
# kind, args, overlay (label, big, small) | painter | None
SH = {
 "o1": ("seq", ("crane", 1.0, None), ("", "", "")),
 "o2": ("seq", ("crane", 1.0, None), ("CHARLES RIVER DAM CROSSING · BOSTON", "61,200", "vehicles a day")),
 "o3": ("seq", ("rovdive", 1.0, None), ("", "", "")),
 "title": ("seq", ("chase", 1.0, None), title_card),
 "p1": ("seq", ("rovblock", 1.0, None, "rcam"), ("", "", "")),
 "p2": ("seq", ("rovdive", 1.0, None, "rcam"), kinetic("30 min", "of bottom time checks about 120 sq ft of pier · FHWA", CYAN)),
 "p3": ("seq", ("rovscour", 1.0, None, "rcam"), kinetic("778", "Massachusetts bridges need an underwater inspection")),
 "p4": ("seq", ("rovblock", 1.0, None, "rcam"), kinetic("7", "full-time divers in MassDOT's dive unit", CYAN)),
 "p5": ("seq", ("rovscour", 1.0, None), ("SCOUR", "", "in a flood, when it matters most, nobody can dive · FHWA: divers lose mobility above ~3 ft/s")),
 "q1": ("card", ("under.png",), lambda d: quote_card(d, ["Sonar could only be used to supplement", "bridge inspection diving operations."], "FHWA guidance, 2009",
                                                     "The 2022 NBIS still requires a qualified diver and team leader at the bridge (23 CFR 650)")),
 "q2": ("card", ("lights.png",), lambda d: statement(d, "What if a robot went first?", "Divers only where the map says.")),
 "b1": ("seq", ("chase", 1.0, None), ("BUILT FROM SCRATCH · ONE DAY", "", "NVIDIA Isaac Sim · real-time ray tracing + physics · a digital twin of this crossing")),
 "b2": ("seq", ("underdeck", 1.0, None), ("REAL AUTOPILOTS IN THE LOOP", "", "ArduCopter flies the drone · ArduSub flies the ROV · ROS 2")),
 "m1": ("hero", ("dronehero", ["lidar"], None), ("MEET THE DRONE · HOLYBRO X500", "", "")),
 "m2": ("hero", ("dronehero", ["lidar", "gimbal", "flood", "gnss"], "therm"), ("MEET THE DRONE · HOLYBRO X500", "", "thermal camera simulated in this demo")),
 "m3": ("hero", ("dronehero", ["lidar", "gimbal"], "therm"), ("THERMAL CAMERA (SIMULATED)", "", "damp, failing stone reads cooler than sound stone")),
 "m4": ("hero", ("dronehero", ["lidar", "gimbal", "flood", "gnss"], None), ("MEET THE DRONE · HOLYBRO X500", "", "")),
 "n1": ("hero", ("rovhero", [], None), ("MEET THE ROV · BLUEROV2 HEAVY", "0", "GPS satellites underwater")),
 "n2": ("hero", ("rovhero", ["dvl"], None), ("MEET THE ROV · BLUEROV2 HEAVY", "0.4 m", "drift after 60 m, no GPS")),
 "n3": ("hero", ("rovhero", ["dvl", "sonar", "camera", "lumen"], "sonar"), ("MULTIBEAM SONAR · 8 THRUSTERS", "", "sees by sound where the camera sees silt")),
 "n4": ("hero", ("rovhero", ["dvl", "sonar", "camera", "lumen"], None), ("8 THRUSTERS", "", "the Heavy holds station against the current")),
 "x1": ("seq", ("chase", 1.0, ["drgb"]), ("THE MISSION", "", "the drone flies the canal wall · lidar + camera + thermal")),
 "x2": ("seq", ("underdeck", 1.0, None), ("UNDER THE DECK", "12 › 7", "GPS satellites: open sky, then under the bridge steel")),
 "x3": ("split", ("chase", "rovdive"), ("THE HANDOFF", "", "damage at the water line · the drone marks it · the ROV is tasked")),
 "x4": ("seq", ("rovdive", 1.0, ["rcam"]), ("STRAIGHT TO THE MARK", "", "ArduSub, navigating on the DVL")),
 "x5": ("seq", ("rovscour", 1.0, ["sonar"]), ("WHAT NO ONE COULD SEE", "", "a missing granite block · a scour pit with exposed timber piles")),
 "x6": ("clip", ("build3/mapbuild.mp4", 0, 1.0, False), ("ONE 3D MODEL · ABOVE + BELOW", "4 / 4", "waterline defects flagged and inspected")),
 "k1": ("dash", (2,), ("THE BRAIN · BUILT WITH KIRO", "", "YOLOv8 finds the defects · every report claim cites one · an engineer signs off")),
 "k2": ("dash", (2,), ("DETECT · CITE · SIGN OFF", "", "YOLOv8 finds defects · every claim cites a detection · an engineer approves")),
 "k3": ("dash", (3,), ("ONE DASHBOARD", "", "3D map · camera footage · GPS track · mission stats · a report Kiro writes itself")),
 "v1": ("card", ("canal.png",), lambda d: money_chart(d, True)),
 "v2": ("card", ("under.png",), kit_chart),
 "v3": ("card", ("basin.png",), lambda d: timeline_card(d, "snbi")),
 "l1": ("seq", ("landing", 1.0, None), ("NEXT · 10-BRIDGE PILOT", "", "simulation today (thermal simulated, waterline flags from ground truth) · next: checked against divers")),
 "c1": ("seq", ("landing", 1.0, None), ("NEXT", "10", "bridges in a pilot · every robot map checked against a diver")),
 "c2": ("seq", ("landing", 1.0, None), end_card),
}
CURSOR = {}


def frames(shot, sub="main"):
    return sorted(glob.glob(RP(shot, sub, "*.jpg")))


def ovl(k, spec):
    im = canvas(); d = ImageDraw.Draw(im)
    if callable(spec): spec(d)
    elif spec and any(spec): lower_third(d, *spec, im)
    if k not in ("title", "c2"): bug(d)
    p = B3("ov", f"{k}.png"); im.save(p); return p


def pipe_frames(out, n, gen):
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{HH}", "-r", str(FPS), "-i", "-"] + ENC + [out],
                         stdin=subprocess.PIPE)
    for i in range(n): p.stdin.write(gen(i).tobytes())
    p.stdin.close(); p.wait()


def grade(im):
    a = np.asarray(im, np.float32); a = (a - 128) * 1.06 + 128; g = a.mean(-1, keepdims=True); a = g + (a - g) * 1.08
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


def seq_frame(shot, sub, idx):
    fs = frames(shot, sub); im = Image.open(fs[min(int(idx), len(fs) - 1)]).convert("RGB")
    return im.resize((W, HH), Image.LANCZOS) if im.size != (W, HH) else im


def seg(k, dur):
    kind, a, spec = SH[k]; ov = Image.open(ovl(k, spec)).convert("RGBA"); out = B3("segs", f"{k}.mp4"); n = int(round(dur * FPS))
    slide = not callable(spec)
    def comp(base, i):
        base = grade(base).convert("RGBA")
        t_ = i / FPS; al = min(t_ / 0.35, 1.0); x = int(-400 + 400 * min(t_ / 0.45, 1.0) * (2 - min(t_ / 0.45, 1.0))) if slide else 0
        o = ov.copy(); o.putalpha(o.getchannel("A").point(lambda v: int(v * al)))
        base.alpha_composite(o, (x, 0) if x > -W else (0, 0)); return base.convert("RGB")
    if kind == "seq":
        shot, sp, pips = a[0], a[1], a[2]; sub = a[3] if len(a) > 3 else "main"
        fs = frames(shot, sub); step = 2 if sub != "main" else 1
        c0 = CURSOR.get((shot, sub), 0)
        if c0 + n * step * sp >= len(fs) * step: c0 = 0
        def gen(i):
            j = int((c0 + i * sp) // step) if sub == "main" else int((c0 + i * sp) // 2)
            im = seq_frame(shot, sub, j)
            for q, ps in enumerate(pips or []):
                pf = frames(shot, ps)
                if pf:
                    p_ = Image.open(pf[min(int((c0 + i * sp) // 2), len(pf) - 1)]).convert("RGB"); p_ = p_.resize((520, int(520 * p_.size[1] / p_.size[0])))
                    fr = Image.new("RGB", (p_.size[0] + 6, p_.size[1] + 6), (110, 205, 255)); fr.paste(p_, (3, 3)); im.paste(fr, (W - fr.size[0] - 64, 110 + q * 330))
            return np.asarray(comp(im, i))
        pipe_frames(out, n, gen); CURSOR[(shot, sub)] = c0 + n * sp * (1 if sub == "main" else 1)
    elif kind == "hero":
        shot, keys, pip = a; fs = frames(shot); cj = {int(kk): v for kk, v in json.load(open(RP(shot, "callouts.json"))).items()}
        ks = sorted(cj); labels = DRONE_CALL if shot == "dronehero" else ROV_CALL
        c0 = CURSOR.get((shot, "main"), 0); rate = min(1.0, (len(fs) - c0 - 1) / max(n, 1)) if c0 < len(fs) - 30 else 0.4
        if c0 >= len(fs) - 30: c0 = 0; rate = min(1.0, (len(fs) - 1) / n)
        new = set(keys) - set(CURSOR.get((shot, "keys"), []))
        def gen(i):
            j = int(c0 + i * rate); im = seq_frame(shot, "main", j).convert("RGBA"); d = ImageDraw.Draw(im)
            kk = min(ks, key=lambda q: abs(q - j)); pts = cj[kk]
            # column layout: labels stacked on one side, ordered by the height of their point -> no box overlaps, no crossings
            right = shot == "dronehero"; f_ = F(BC, 36); order = sorted(keys, key=lambda nm: pts[nm][1])
            slots = {nm: 170 + 92 * q for q, nm in enumerate([o for o in order if o != "dvl"])}; slots["dvl"] = 0                  # y slots 170..446: above the feed and the caption
            for m_, name in enumerate(order):
                x, y = pts[name]; grow = 1.0 if name not in new else min(max((i / FPS - 0.25 * keys.index(name)) / 0.6, 0), 1)
                if grow <= 0: continue
                tx = labels[name]; tw = d.textlength(tx, font=f_); ly = slots[name]
                bx0 = 1290 if right else 90; bx1 = bx0 + tw + 28
                ax = bx0 if right else bx1                                             # leader attaches to the near edge of the box
                if name == "dvl":                                                      # under the hull: label straight below it, short vertical leader
                    ly = y + 110; bx0 = x - (tw + 28) / 2; bx1 = bx0 + tw + 28; ax = x
                d.ellipse([x - 9, y - 9, x + 9, y + 9], outline=AMBER + (255,), width=4)
                ty = ly - 36 if name == "dvl" else ly - 8
                g = min(grow * 1.6, 1.0); d.line([(x, y), (x + (ax - x) * g, y + (ty - y) * g)], fill=AMBER + (255,), width=3)
                if grow >= 0.6:
                    d.rounded_rectangle([bx0, ly - 36, bx1, ly + 14], 8, fill=(6, 12, 20, 225), outline=AMBER + (160,), width=2)
                    d.text((bx0 + 14, ly + 2), tx, font=f_, fill=WHITE + (255,), anchor="ls")
            if pip:
                src = {"therm": B3("capA_therm.mp4"), "sonar": None}[pip] if pip == "therm" else None
                pf = frames("rovblock", "sonar") if pip == "sonar" else None
                if pip == "sonar" and pf:
                    p_ = Image.open(pf[min(i // 2, len(pf) - 1)]).convert("RGB").resize((460, 246)); im.paste(p_, (W - 524, HH - 246 - 150))
                if pip == "therm" and THERM:
                    p_ = THERM[min(i // 15, len(THERM) - 1)].resize((420, 336)); im.paste(p_, (W - 484, HH - 336 - 150))
            return np.asarray(comp(im.convert("RGB"), i))
        pipe_frames(out, n, gen); CURSOR[(shot, "main")] = c0 + n * rate; CURSOR[(shot, "keys")] = keys
    elif kind == "split":
        top, bot = a
        ct, cb = CURSOR.get((top, "main"), 0), CURSOR.get((bot, "main"), 0)
        def gen(i):
            t_ = seq_frame(top, "main", ct + i); b_ = seq_frame(bot, "main", cb + i)
            im = Image.new("RGB", (W, HH)); im.paste(t_.crop((0, 270, W, 810)), (0, 0)); im.paste(b_.crop((0, 270, W, 810)), (0, 540))
            d = ImageDraw.Draw(im); d.rectangle([0, 536, W, 544], fill=(110, 205, 255))
            d.text((64, 520), "ABOVE · DRONE", font=F(BC, 40), fill=WHITE, anchor="ls"); d.text((64, 600), "BELOW · ROV", font=F(BC, 40), fill=WHITE, anchor="ls")
            return np.asarray(comp(im, i))
        pipe_frames(out, n, gen); CURSOR[(top, "main")] = ct + n; CURSOR[(bot, "main")] = cb + n
    elif kind == "card":
        img = os.path.join(ST, a[0]); ovp = B3("ov", f"{k}.png")
        fc = (f"[0:v]scale=3840:2160:force_original_aspect_ratio=increase,crop=3840:2160,zoompan=z='1.0+0.05*on/{n}':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d={n}:s={W}x{HH}:fps={FPS},gblur=sigma=22,eq=brightness=-0.10[b];"
              f"[1:v]format=rgba,fade=t=in:st=0:d=0.35:alpha=1[o];[b][o]overlay=0:0[v]")
        ff(["-loop", "1", "-i", img, "-loop", "1", "-i", ovp, "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind in ("clip", "dash"):
        if kind == "clip": src, t0, sp, crop = a; src = P(src)
        else: src, t0, sp, crop = DASH[a[0]], 0, 1.0, False
        ovp = B3("ov", f"{k}.png"); cr = "crop=1416:797:468:140," if crop else ""
        fc = f"[0:v]setpts=PTS/{sp},{cr}{BASE}[m];[1:v]format=rgba,fade=t=in:st=0:d=0.35:alpha=1[o];[m][o]overlay=0:0[v]"
        ff(["-ss", str(t0), "-stream_loop", "-1", "-i", src, "-loop", "1", "-i", ovp, "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    return out


os.makedirs(B3("therm"), exist_ok=True)
ff(["-ss", "20", "-i", B3("capA_therm.mp4"), "-vf", "fps=2", "-frames:v", "12", B3("therm", "t%02d.png")])
THERM = [Image.open(f_).convert("RGB").resize((520, 416)) for f_ in sorted(glob.glob(B3("therm", "t*.png")))]

starts = [("o1", 0.0)] + [(k, t0) for k, t0, d in TL[1:]] + [("title", TITLE[0])]
starts = sorted([(k, (TITLE[0] + TITLE[1]) if k == "p1" else s) for k, s in starts], key=lambda x: x[1])
json.dump(dict(vo=TL, total=TOTAL, title=TITLE, starts=starts), open(B3("timeline.json"), "w"), indent=1)
if os.environ.get("PREVIEW"):
    for k in os.environ["PREVIEW"].split(","): print(seg(k, 6.0))
    raise SystemExit
segs = []
for i, (k, s0) in enumerate(starts):
    e = starts[i + 1][1] if i + 1 < len(starts) else TOTAL
    segs.append((k, s0, seg(k, e - s0 + (XF if i + 1 < len(starts) else 0)))); print(f"{k:6s} {s0:6.1f}s {e - s0:5.2f}s", flush=True)
ins, fc, last = [], "", "0:v"
for k, s0, f in segs: ins += ["-i", f]
for i in range(1, len(segs)):
    k, s0, _ = segs[i]
    tr = "fadeblack" if k in ("title", "c2", "q1") else ("smoothleft" if k in ACTS else "fade")
    fc += f"[{last}][{i}:v]xfade=transition={tr}:duration={XF}:offset={s0:.3f}[x{i}];"; last = f"x{i}"
ff(ins + ["-filter_complex", fc.rstrip(";"), "-map", f"[{last}]", "-t", f"{TOTAL:.3f}"] + ENC + [B3("video_noaudio.mp4")])
print("VIDEO", f"{TOTAL:.1f}s", len(segs), "segments")
