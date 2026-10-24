"""3-minute project film: VO timeline -> per-line segments (captured sim footage, path-traced stills, number cards) with
PIL overlays -> build/segs/*.mp4 -> build/video_noaudio.mp4. Mix + mux: assemble.py.
Footage is the real-time sim capture (build/<stream>.mp4 from streams.py) and the RViz screen recording (cap/rviz.mp4).
"""
import os, json, subprocess, sys
import numpy as np, soundfile as sf
from PIL import Image, ImageDraw, ImageFont, ImageFilter
H = os.path.dirname(os.path.abspath(__file__)); P = lambda *a: os.path.join(H, *a)
ST = os.path.join(H, "..", "data", "stills")
FD = os.path.expanduser("~/biofouling-yc/video/assets/")
F = lambda n, s: ImageFont.truetype(FD + n, s)
W, HH, FPS = 1920, 1080, 30
GAP, LEAD = 0.75, 4.2
PAUSE = {"h2": 1.6, "c1": 2.0, "b1": 1.0, "d4": 1.6, "r4": 1.8, "t4": 2.2, "a2": 1.6, "l1": 0.8}   # music-only beats between sections
os.makedirs(P("build", "segs"), exist_ok=True); os.makedirs(P("build", "ov"), exist_ok=True)
CAP = dict(json.load(open(P("script_captions.json"))))
ORDER = [k for k, _ in json.load(open(P("script_captions.json")))]

# ---------------------------------------------------------------- timeline from the VO takes
t, TL = LEAD, []
for k in ORDER:
    d = sf.info(P("vo", f"{k}.wav")).duration
    TL.append((k, t, d)); t += d + GAP + PAUSE.get(k, 0.0)
TOTAL = t + 3.5                                                            # end card holds
json.dump(dict(vo=TL, total=TOTAL), open(P("build", "timeline.json"), "w"), indent=1)

# ---------------------------------------------------------------- shots: (kind, args, section label, big number, small text)
SH = {
 "h1": ("still", ("canal.png", 1.0, 1.08), "THE SITE", "61,200", "vehicles a day · Charles River Dam crossing, Boston"),
 "h2": ("still", ("basin.png", 1.06, 1.0), "NBI 2022 RATING", "4 / 9", "substructure · channel protection  =  POOR"),
 "p1": ("card", ("basin.png",), "MASSACHUSETTS · NBI 2022", "444", "of 5,252 bridges rated Poor"),
 "p2": ("card", ("canal.png",), "MASSACHUSETTS · NBI 2022", "10.1 M", "vehicle crossings a day on Poor bridges"),
 "p3": ("card", ("drone.png",), "UNDER THE WATER", "778", "bridges need diver inspection · 123 scour critical"),
 "c1": ("clip", ("rchase", 90, 1.0, None), "SCOUR", "", "the river eats the bed around the foundation"),
 "b1": ("rviz", (100, 3.0), "BUILT IN ONE DAY", "", "Isaac Sim · ArduPilot SITL · ROS 2 · RViz2"),
 "d1": ("clip", ("dchase", 2, 2.5, None), "DRONE · HOLYBRO X500", "38", "MAVLink mission items · ArduCopter in AUTO"),
 "d2": ("clip", ("dchase", 20, 2.0, ["drgb", "therm"]), "SENSORS", "", "Ouster OS0-128 lidar · gimbal RGB · thermal (simulated)"),
 "d3": ("clip", ("drgb", 44, 1.2, None), "UNDER THE DECK", "12 to 7", "GNSS satellites: open sky, then under the bridge"),
 "d4": ("clip", ("dchase", 56, 2.0, ["drgb"]), "", "", "GPS fails exactly where inspection matters"),
 "r1": ("clip", ("rchase", 14, 1.4, None), "ROV · BLUEROV2 HEAVY", "0", "GPS satellites · real ArduSub SITL"),
 "r2": ("clip", ("rchase", 26, 1.4, ["rcam"]), "DVL NAVIGATION", "", "Doppler velocity log feeds the ArduSub EKF (BlueOS-style)"),
 "r3": ("split", ("sonar", "rcam", 40, 1.2), "MULTIBEAM SONAR + CAMERA", "130°", "12 m sonar range · turbid-water camera"),
 "r4": ("clip", ("rchase", 48, 1.3, ["sonar"]), "MEASURED DRIFT", "0.4 m", "after 60 m of travel, no GPS"),
 "t1": ("clip", ("dchase", 6, 2.0, ["rchase"]), "WORKING TOGETHER", "", "drone flags defects at the water line"),
 "t2": ("clip", ("rchase", 58, 1.3, ["rcam"]), "AUTO-TASKED", "", "ROV dives to the wall directly below each flag"),
 "t3": ("rviz", (150, 2.5), "OUR RECORDED RUN", "4 / 4", "waterline defects flagged, all inspected underwater"),
 "t4": ("still", ("rviz_3d_crop.png", 1.0, 1.08), "ONE 3D MODEL", "4.54 M", "2.96 M lidar points above + 1.58 M sonar points below"),
 "a1": ("triage", (), "AI TRIAGE · BUILT WITH KIRO", "5,252", "Massachusetts bridges scored for underwater risk"),
 "a2": ("triage", (), "#1 RISK", "", "North Washington St bridge over the Charles · scour critical"),
 "l1": ("card", ("rviz_drone_rov.png",), "HONEST LIMITS", "", "simulation · simulated thermal · flags from ground truth"),
 "c2": ("still", ("lights.png", 1.0, 1.1), "", "", ""),
 "title": ("still", ("canal.png", 1.12, 1.0), "", "", ""),
}


def overlay(k, label, big, small, extra=None):
    im = Image.new("RGBA", (W, HH), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
    if label or big or small:                                              # lower-left block on a soft gradient
        g = Image.new("L", (W, 420)); gd = ImageDraw.Draw(g)
        for y in range(420): gd.line([(0, y), (W, y)], fill=int(170 * (y / 420) ** 1.6))
        im.paste((5, 10, 16, 255), (0, HH - 420), g)
        y = HH - 90
        if small: d.text((96, y), small, font=F("Barlow-Regular.ttf", 38), fill=(225, 232, 240, 255), anchor="ls"); y -= 58
        if big: d.text((90, y), big, font=F("BarlowCondensed-SemiBold.ttf", 150), fill=(255, 196, 64, 255), anchor="ls"); y -= 160
        if label: d.text((96, y), label, font=F("BarlowCondensed-SemiBold.ttf", 40), fill=(120, 200, 255, 255), anchor="ls")
    if extra: extra(d)
    d.text((W - 60, 60), "HACKACHUSETTS 2026 · R6", font=F("BarlowCondensed-SemiBold.ttf", 30), fill=(255, 255, 255, 150), anchor="rs")
    p = P("build", "ov", f"{k}.png"); im.save(p); return p


def triage_table(d):
    rows = [("1", "80.1", "N Washington St / Charles River", "scour critical"), ("2", "80.1", "N Washington St / Charles River", "scour critical"),
            ("3", "80.1", "N Washington St / Charles River", "scour critical"), ("4", "78.9", "US 44 Winthrop St / Palmer River", "scour critical"),
            ("5", "78.5", "Elm St / Sudbury River", "scour critical")]
    x0, y0 = 120, 150
    d.rounded_rectangle([x0 - 30, y0 - 40, x0 + 1250, y0 + 360], 18, fill=(8, 14, 22, 215))
    d.text((x0, y0), "RANK   SCORE   BRIDGE (NBI 2022)                                      REASON", font=F("IBMPlexMono-Medium.ttf", 26), fill=(120, 200, 255, 255))
    for i, (r, s, b, why) in enumerate(rows):
        y = y0 + 60 + i * 56; col = (255, 196, 64, 255) if i == 0 else (225, 232, 240, 255)
        d.text((x0, y), f" {r}     {s}   {b:<44} {why}", font=F("IBMPlexMono-Regular.ttf", 28), fill=col)


def end_card(d):
    d.rectangle([0, 0, W, HH], fill=(4, 8, 14, 150))
    d.text((W // 2, 430), "Above and below the waterline.", font=F("BarlowCondensed-SemiBold.ttf", 110), fill=(255, 255, 255, 255), anchor="ms")
    d.text((W // 2, 520), "One pass. One model.", font=F("BarlowCondensed-SemiBold.ttf", 80), fill=(255, 196, 64, 255), anchor="ms")
    d.text((W // 2, 640), "Drone (ArduCopter) + BlueROV2 (ArduSub, DVL) · Isaac Sim · ROS 2 · AI triage with Kiro", font=F("Barlow-Regular.ttf", 36), fill=(210, 220, 230, 255), anchor="ms")
    d.text((W // 2, 700), "Craigie Drawbridge & Charles River Dam crossing, Boston · github.com/mochi-bunny/navfix", font=F("Barlow-Regular.ttf", 32), fill=(160, 180, 200, 255), anchor="ms")
    d.text((W // 2, 800), "Built at Sundai · Hackachusetts 2026", font=F("BarlowCondensed-SemiBold.ttf", 44), fill=(120, 200, 255, 255), anchor="ms")


def title_card(d):
    d.rectangle([0, 0, W, HH], fill=(4, 8, 14, 110))
    d.text((W // 2, 470), "ABOVE & BELOW", font=F("BarlowCondensed-SemiBold.ttf", 170), fill=(255, 255, 255, 255), anchor="ms")
    d.text((W // 2, 560), "A drone and an underwater robot inspect one bridge, together", font=F("Barlow-Regular.ttf", 46), fill=(225, 232, 240, 255), anchor="ms")
    d.text((W // 2, 640), "Craigie Drawbridge · Charles River Dam · Boston", font=F("BarlowCondensed-SemiBold.ttf", 40), fill=(255, 196, 64, 255), anchor="ms")


def ff(args):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + args, check=True)


ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "19", "-pix_fmt", "yuv420p", "-r", str(FPS)]
BASE = f"scale={W}:{HH}:force_original_aspect_ratio=increase,crop={W}:{HH},setsar=1"


def seg(i, k, dur):
    kind, a, label, big, small = SH[k]
    extra = triage_table if kind == "triage" and k == "a1" else (end_card if k == "c2" else (title_card if k == "title" else None))
    ov = overlay(k, label, big, small, extra)
    out = P("build", "segs", f"{i:02d}_{k}.mp4"); n = int(round(dur * FPS))
    if kind in ("still", "card", "triage"):
        img = os.path.join(ST, a[0]) if kind != "triage" else os.path.join(ST, "rviz_drone_rov.png")
        z0, z1 = (a[1], a[2]) if kind == "still" else (1.0, 1.04)
        blur = ",boxblur=18:2,eq=brightness=-0.12" if kind in ("card", "triage") else ""
        vf = (f"[0:v]scale=3840:2160:force_original_aspect_ratio=increase,crop=3840:2160,zoompan=z='{z0}+({z1}-{z0})*on/{n}':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d={n}:s={W}x{HH}:fps={FPS}{blur}[b];"
              f"[b][1:v]overlay=0:0[v]")
        ff(["-loop", "1", "-i", img, "-loop", "1", "-i", ov, "-filter_complex", vf, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "clip":
        st, t0, sp, pips = a
        ins = ["-ss", str(t0), "-i", P("build", f"{st}.mp4")]; fc = f"[0:v]setpts=PTS/{sp},{BASE}[m];"; last = "m"
        for j, ps in enumerate(pips or []):
            ins += ["-ss", str(t0), "-i", P("build", f"{ps}.mp4")]
            fc += f"[{j + 1}:v]setpts=PTS/{sp},scale=560:-2,pad=iw+6:ih+6:3:3:color=0xFFC440[p{j}];[{last}][p{j}]overlay=W-w-60:{110 + j * 340}[o{j}];"; last = f"o{j}"
        ins += ["-loop", "1", "-i", ov]
        fc += f"[{last}][{len(pips or []) + 1}:v]overlay=0:0[v]"
        ff(ins + ["-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "split":
        a1, a2, t0, sp = a
        fc = (f"[0:v]setpts=PTS/{sp},scale=960:-2,pad=960:{HH}:0:(oh-ih)/2:color=0x05080C[l];[1:v]setpts=PTS/{sp},scale=960:-2,pad=960:{HH}:0:(oh-ih)/2:color=0x05080C[r];"
              f"[l][r]hstack[m];[m][2:v]overlay=0:0[v]")
        ff(["-ss", str(t0), "-i", P("build", f"{a1}.mp4"), "-ss", str(t0), "-i", P("build", f"{a2}.mp4"), "-loop", "1", "-i", ov,
            "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "rviz":
        t0, sp = a
        fc = f"[0:v]setpts=PTS/{sp},crop=1416:797:468:140,{BASE}[m];[m][1:v]overlay=0:0[v]"
        ff(["-ss", str(t0), "-i", P("cap", "rviz.mp4"), "-loop", "1", "-i", ov, "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    return out


segs = [seg(99, "title", TL[0][1])]
for i, (k, t0, d) in enumerate(TL):
    nxt = TL[i + 1][1] if i + 1 < len(TL) else TOTAL
    start = t0
    segs.append(seg(i, k, nxt - start)); print(k, f"{nxt - start:.2f}s", flush=True)
with open(P("build", "segs.txt"), "w") as f:
    for s in segs: f.write(f"file '{s}'\n")
ff(["-f", "concat", "-safe", "0", "-i", P("build", "segs.txt"), "-c", "copy", P("build", "video_noaudio.mp4")])
print("VIDEO", f"{TOTAL:.1f}s")
