"""Production cut v2 (~3:20): smooth real-time captures (capA drone, capB ROV + RViz), Megha's Kiro dashboard recording,
designed cards (pain numbers, FHWA quote, money chart, why-now timeline), xfade transitions, slide-in lower thirds.
    ~/biofouling-yc/voice/.venv/bin/python edit2.py       -> build2/video_noaudio.mp4 + build2/timeline.json
"""
import os, json, subprocess
import soundfile as sf
from PIL import Image, ImageDraw, ImageFont
H = os.path.dirname(os.path.abspath(__file__)); P = lambda *a: os.path.join(H, *a)
B = lambda *a: os.path.join(H, "build2", *a)
ST = os.path.join(H, "..", "data", "stills")
FD = os.path.expanduser("~/biofouling-yc/video/assets/")
F = lambda n, s: ImageFont.truetype(FD + n, s)
BC, BR, BL, MONO = "BarlowCondensed-SemiBold.ttf", "Barlow-Regular.ttf", "BarlowCondensed-Light.ttf", "IBMPlexMono-Medium.ttf"
W, HH, FPS, XF = 1920, 1080, 30, 0.5
AMBER, CYAN, WHITE, GREY, RED = (255, 196, 64), (110, 205, 255), (240, 244, 248), (170, 182, 196), (255, 96, 80)
os.makedirs(B("segs"), exist_ok=True); os.makedirs(B("ov"), exist_ok=True)
SCRIPT = json.load(open(P("script2_captions.json"))); ORDER = [k for k, _ in SCRIPT]

# ---------------------------------------------------------------- timeline: VO lines + breathing room between acts
GAP, LEAD = 0.55, 3.2
PAUSE = {"o2": 3.4, "p5": 1.6, "i3": 2.4, "s3": 1.0, "d3": 0.8, "r3": 1.0, "t3": 2.4, "k5": 1.6, "m3": 1.4, "w3": 1.4}
t, TL = LEAD, []
for k in ORDER:
    d = sf.info(P("vo2", f"{k}.wav")).duration
    TL.append((k, t, d)); t += d + GAP + PAUSE.get(k, 0.0)
TOTAL = t + 6.0
ACT = {"p1": "pain", "i1": "insight", "s1": "build", "d1": "drone", "r1": "rov", "t1": "together", "k1": "ai", "m1": "money", "w1": "now", "c1": "close"}
json.dump(dict(vo=TL, total=TOTAL, acts={k: v for k, v in ACT.items()}, title=[TL[1][1] + TL[1][2] + 0.3, 3.0]), open(B("timeline.json"), "w"), indent=1)

# ---------------------------------------------------------------- overlay helpers (RGBA PNG per segment)
def canvas(): return Image.new("RGBA", (W, HH), (0, 0, 0, 0))


def lower_third(d, label, big, small, im):
    if not (label or big or small): return
    g = Image.new("L", (W, 460)); gd = ImageDraw.Draw(g)
    for y in range(460): gd.line([(0, y), (W, y)], fill=int(185 * (y / 460) ** 1.7))
    im.paste((4, 9, 15, 255), (0, HH - 460), g)
    y = HH - 86
    if small: d.text((100, y), small, font=F(BR, 38), fill=WHITE + (255,), anchor="ls"); y -= 56
    if big: d.text((94, y), big, font=F(BC, 156), fill=AMBER + (255,), anchor="ls"); y -= 166
    if label:
        d.rectangle([100, y - 30, 108, y + 4], fill=CYAN + (255,)); d.text((122, y), label, font=F(BC, 40), fill=CYAN + (255,), anchor="ls")


def bug(d):
    d.text((W - 64, 64), "ABOVE & BELOW · HACKACHUSETTS 2026", font=F(BC, 28), fill=(255, 255, 255, 140), anchor="rs")


def card_bg(d, a=200):
    d.rectangle([0, 0, W, HH], fill=(5, 9, 15, a))


def stat_card(d, big, line1, line2, src, color=AMBER):
    card_bg(d)
    d.text((W // 2, 520), big, font=F(BC, 300), fill=color + (255,), anchor="ms")
    d.text((W // 2, 610), line1, font=F(BR, 54), fill=WHITE + (255,), anchor="ms")
    if line2: d.text((W // 2, 680), line2, font=F(BR, 40), fill=GREY + (255,), anchor="ms")
    d.text((W // 2, HH - 70), src, font=F(BR, 26), fill=(130, 145, 160, 255), anchor="ms")


def quote_card(d, quote, who, src):
    card_bg(d, 215)
    d.text((180, 330), "“", font=F(BC, 260), fill=AMBER + (255,), anchor="ls")
    y = 420
    for ln in quote:
        d.text((260, y), ln, font=F(BL, 72), fill=WHITE + (255,), anchor="ls"); y += 92
    d.text((262, y + 40), who, font=F(BC, 42), fill=CYAN + (255,), anchor="ls")
    d.text((262, y + 92), src, font=F(BR, 28), fill=(140, 155, 170, 255), anchor="ls")


def statement(d, l1, l2, l3=""):
    card_bg(d, 225)
    d.text((W // 2, 450), l1, font=F(BC, 150), fill=WHITE + (255,), anchor="ms")
    d.text((W // 2, 600), l2, font=F(BC, 120), fill=AMBER + (255,), anchor="ms")
    if l3: d.text((W // 2, 700), l3, font=F(BR, 44), fill=GREY + (255,), anchor="ms")


def money_chart(d, highlight):
    card_bg(d, 225)
    d.text((160, 200), "COST TO INSPECT ONE BRIDGE UNDERWATER", font=F(BC, 48), fill=CYAN + (255,), anchor="ls")
    x0, s = 160, 1150 / 12000
    rows = [("Divers only, today", 6800, 10800, RED, "$6.8k – $10.8k", "one dive-team day (FHWA field study, 2012 $)"),
            ("Robot first, then a targeted dive", 5300, 7600, AMBER, "$5.3k – $7.6k", "~$3–4k survey + a third of a dive day (our estimate)")]
    for i, (lab, a_, b_, col, val, note) in enumerate(rows):
        y = 350 + i * 270
        d.text((x0, y - 22), lab, font=F(BC, 46), fill=WHITE + (255,), anchor="ls")
        d.rectangle([x0, y, x0 + int(b_ * s), y + 76], fill=col + (90,)); d.rectangle([x0, y, x0 + int(a_ * s), y + 76], fill=col + (255,))
        d.text((x0 + int(b_ * s) + 30, y + 58), val, font=F(BC, 64), fill=col + (255,), anchor="ls")
        d.text((x0, y + 122), note, font=F(BR, 30), fill=GREY + (255,), anchor="ls")
    for v in range(0, 12001, 4000):
        d.line([(x0 + int(v * s), 880), (x0 + int(v * s), 900)], fill=GREY + (255,), width=2)
        d.text((x0 + int(v * s), 940), f"${v // 1000}k", font=F(BR, 30), fill=GREY + (255,), anchor="ms")
    if highlight:
        d.rounded_rectangle([1180, 950, 1800, 1030], 14, fill=AMBER + (255,))
        d.text((1490, 1005), "≈ 20–30% less per bridge (our estimate)", font=F(BC, 40), fill=(10, 14, 20, 255), anchor="ms")


def kit_chart(d):
    card_bg(d, 225)
    d.text((160, 200), "THE HARDWARE", font=F(BC, 48), fill=CYAN + (255,), anchor="ls")
    items = [("BlueROV2", 4900), ("Heavy kit + tariff", 1008), ("Ping360 sonar", 2650), ("DVL A50", 7890)]
    y = 300
    for n, v in items:
        d.text((160, y), n, font=F(BR, 40), fill=WHITE + (255,), anchor="ls"); d.text((760, y), f"${v:,}", font=F(MONO, 38), fill=WHITE + (255,), anchor="rs"); y += 62
    d.line([(160, y - 30), (760, y - 30)], fill=GREY + (255,), width=2)
    d.text((160, y + 30), "Our kit, list price", font=F(BC, 48), fill=AMBER + (255,), anchor="ls"); d.text((760, y + 30), "$16,448", font=F(BC, 64), fill=AMBER + (255,), anchor="rs")
    d.text((1000, 360), "FHWA-tested sonar systems", font=F(BR, 40), fill=WHITE + (255,), anchor="ls")
    d.text((1000, 470), "$35k – $385k", font=F(BC, 120), fill=RED + (255,), anchor="ls")
    d.text((1000, 530), "for the sonar alone", font=F(BR, 36), fill=GREY + (255,), anchor="ls")
    d.text((1000, 640), "Priced with a Ping360 scanning sonar; the sim's 130° multibeam", font=F(BR, 32), fill=GREY + (255,), anchor="ls")
    d.text((1000, 684), "is a pricier class and is not in this total.", font=F(BR, 32), fill=GREY + (255,), anchor="ls")
    d.text((W // 2, HH - 70), "Blue Robotics list prices, Oct 2026 · FHWA-HIF-18-049 · before tether, laptop, boat, tax, shipping",
           font=F(BR, 26), fill=(130, 145, 160, 255), anchor="ms")


def timeline_card(d, focus):
    card_bg(d, 225)
    d.text((160, 200), "WHY NOW", font=F(BC, 48), fill=CYAN + (255,), anchor="ls")
    y0 = 560; d.line([(200, y0), (1720, y0)], fill=GREY + (255,), width=4)
    ev = [(260, "2023 – 2026", "New Hampshire DOT", "$183k ROV study", "nh"), (900, "TODAY", "Pilot money exists", "FHWA STIC $125k/yr · AID up to $1M", "money"),
          (1600, "15 MAR 2028", "SNBI deadline", "every state re-codes its bridges, incl. scour", "snbi")]
    for x, when, a, b, key in ev:
        col = AMBER if key == focus else WHITE
        d.ellipse([x - 16, y0 - 16, x + 16, y0 + 16], fill=col + (255,))
        d.text((x, y0 - 50), when, font=F(BC, 54), fill=col + (255,), anchor="ms")
        d.text((x, y0 + 80), a, font=F(BC, 46), fill=col + (255,), anchor="ms")
        d.text((x, y0 + 130), b, font=F(BR, 32), fill=GREY + (255,), anchor="ms")


def kiro_card(d):
    card_bg(d, 210)
    d.text((W // 2, 360), "THE AI LAYER, BUILT WITH KIRO", font=F(BC, 96), fill=WHITE + (255,), anchor="ms")
    d.text((W // 2, 430), "Kiro: AWS's agentic AI coding IDE", font=F(BR, 42), fill=GREY + (255,), anchor="ms")
    cols = [("DETECT", "YOLOv8-seg defect model\non drone imagery"), ("RETRACE", "every hit › the recorded\nlidar at that pose"),
            ("JUDGE", "severity by rules,\nnot by the LLM"), ("REPORT", "agent cites a detection\nfor every claim · PE signs off")]
    for i, (h, t) in enumerate(cols):
        x = 250 + i * 470
        d.rounded_rectangle([x - 200, 520, x + 200, 760], 18, outline=CYAN + (255,), width=3, fill=(10, 18, 28, 230))
        d.text((x, 590), h, font=F(BC, 54), fill=AMBER + (255,), anchor="ms")
        for j, ln in enumerate(t.split("\n")): d.text((x, 650 + j * 44), ln, font=F(BR, 32), fill=WHITE + (255,), anchor="ms")
    d.text((W // 2, 880), "“Code enforces, models explain.”  — Megha, detection pipeline + dashboard", font=F(BR, 38), fill=CYAN + (255,), anchor="ms")


def triage_card(d):
    card_bg(d, 225)
    d.text((160, 200), "STATEWIDE TRIAGE · 5,252 MASSACHUSETTS BRIDGES · NBI 2022", font=F(BC, 48), fill=CYAN + (255,), anchor="ls")
    rows = [("1", "80.1", "N Washington St over the Charles River", "scour critical"), ("2", "78.9", "US 44 Winthrop St over the Palmer River", "scour critical"),
            ("3", "78.5", "Elm St over the Sudbury River", "scour critical")]
    for i, (r, s, b, why) in enumerate(rows):
        y = 330 + i * 110; col = AMBER if i == 0 else WHITE
        d.text((160, y), f"#{r}", font=F(BC, 80), fill=col + (255,), anchor="ls"); d.text((300, y), b, font=F(BR, 48), fill=col + (255,), anchor="ls")
        d.text((1380, y), f"{s}  ·  {why}", font=F(MONO, 30), fill=GREY + (255,), anchor="ls")
    d.text((160, 760), "#1 was a mile from our site. Boston has since replaced it (Bill Russell Bridge, 2023–25).", font=F(BC, 54), fill=AMBER + (255,), anchor="ls")
    d.text((160, 840), "Risk score from condition, scour rating, age and traffic. Transparent weights, one line of code to change.", font=F(BR, 34), fill=GREY + (255,), anchor="ls")


def title_card(d):
    card_bg(d, 120)
    d.text((W // 2, 300), "CRAIGIE BRIDGE INSPECTOR", font=F(BC, 54), fill=AMBER + (255,), anchor="ms")
    d.text((W // 2, 520), "ABOVE & BELOW", font=F(BC, 230), fill=WHITE + (255,), anchor="ms")
    d.text((W // 2, 610), "A drone and an underwater robot inspect one bridge, together", font=F(BR, 50), fill=WHITE + (255,), anchor="ms")


def end_card(d):
    card_bg(d, 235)
    d.text((W // 2, 300), "Robot first.", font=F(BC, 150), fill=WHITE + (255,), anchor="ms")
    d.text((W // 2, 440), "Divers only where the map says.", font=F(BC, 110), fill=AMBER + (255,), anchor="ms")
    lines = [("Ferbin · robots, simulation, ArduPilot + ROS 2", WHITE), ("Megha · detection AI + inspector dashboard, built with Kiro", WHITE),
             ("Thiago · business case and market validation", WHITE)]
    for i, (t, c) in enumerate(lines): d.text((W // 2, 570 + i * 56), t, font=F(BR, 40), fill=c + (255,), anchor="ms")
    d.text((W // 2, 790), "NVIDIA Isaac Sim · ArduCopter + ArduSub SITL · ROS 2 Jazzy · Kiro · github.com/mochi-bunny/navfix · /detectfix",
           font=F(BR, 30), fill=GREY + (255,), anchor="ms")
    d.text((W // 2, 840), "Sources: FHWA NBI 2022, FHWA-HIF-18-049, 23 CFR 650, Blue Robotics. Estimates are ours. Simulation; thermal simulated.",
           font=F(BR, 26), fill=(130, 145, 160, 255), anchor="ms")
    d.text((W // 2, 885), "Music: “Lightless Dawn”, “Impact Prelude”, “Discovery Hit” by Kevin MacLeod (incompetech.com), CC BY 4.0",
           font=F(BR, 26), fill=(130, 145, 160, 255), anchor="ms")
    d.text((W // 2, 960), "Built at Sundai · Hackachusetts 2026 · Robotics track", font=F(BC, 44), fill=CYAN + (255,), anchor="ms")


NBI = "FHWA National Bridge Inventory 2022, Massachusetts"
# kind, source, (label, big, small) or a card painter
SH = {
 "o1": ("still", ("canal.png", 1.0, 1.10), ("CHARLES RIVER DAM CROSSING · BOSTON", "61,200", "vehicles a day")),
 "o2": ("clip", ("capB_rchase", 2, 1.0, None), ("", "", "")),
 "p1": ("card", ("basin.png",), lambda d: stat_card(d, "778", "Massachusetts bridges need underwater inspection", "NBI item 92B", NBI)),
 "p2": ("card", ("canal.png",), lambda d: stat_card(d, "123", "are scour critical", "1.6 million vehicles cross them every day", NBI)),
 "p3": ("card", ("under.png",), lambda d: stat_card(d, "7", "full-time divers in MassDOT's dive unit", "", "Boston 25 News, 2025", CYAN)),
 "p4": ("card", ("basin.png",), lambda d: stat_card(d, "$6.8k–10.8k", "for one dive-team day, per bridge", "", "FHWA-HIF-18-049, Table 9 (2012 dollars)", RED)),
 "p5": ("clip", ("capB_rchase", 222, 0.8, None), ("SCOUR", "", "the river eats the bed around the foundation · in a flood, nobody can dive")),
 "i1": ("card", ("under.png",), lambda d: quote_card(d, ["Sonar could only be used to supplement", "bridge inspection diving operations."], "FHWA guidance, 2009",
                                                     "The 2022 NBIS still requires a qualified diver and team leader at the bridge (23 CFR 650)")),
 "i2": ("card", ("canal.png",), lambda d: quote_card(d, ["I'd send an ROV down, give a visual of the", "entire basin… send divers in and do", "specific spot checks."],
                                                      "A federal dam inspector, this week", "discovery call, Oct 2026 (anonymous)")),
 "i3": ("card", ("lights.png",), lambda d: statement(d, "Divers stay. The robot goes first.", "Divers only where the map says.")),
 "s1": ("still", ("boat.png", 1.08, 1.0), ("BUILT FROM SCRATCH AT THE EVENT", "TODAY", "NVIDIA Isaac Sim: real-time ray tracing + physics · a digital twin to test robots before the river")),
 "s2": ("still", ("basin.png", 1.0, 1.08), ("DIGITAL TWIN", "1910", "Craigie drawbridge + 1906 dam bridge, rebuilt from the NBI record and photos")),
 "s3": ("clip", ("capA_dchase", 1, 1.2, ["capB_rchase"]), ("REAL AUTOPILOTS IN THE LOOP", "", "ArduCopter + ArduSub SITL · MAVLink · ROS 2, 21 topics recorded")),
 "d1": ("clip", ("capA_dchase", 8, 1.2, ["capA_drgb", "capA_therm"]), ("DRONE · HOLYBRO X500", "", "Ouster OS0-128 lidar · gimbal RGB · thermal (simulated) · floodlights")),
 "d2": ("still", ("girders.png", 1.0, 1.10), ("UNDER THE DECK", "12 to 7", "GPS satellites: open sky, then under the bridge steel")),
 "d3": ("clip", ("capA_dchase", 36, 1.2, ["capA_drgb"]), ("ABOVE THE WATER LINE", "", "the drone flags what it sees at the water line")),
 "r1": ("clip", ("capB_rchase", 4, 1.0, None), ("ROV · BLUEROV2 HEAVY", "0", "GPS satellites underwater · real ArduSub SITL")),
 "r2": ("clip", ("capB_rchase", 46, 1.0, ["capB_rcam"]), ("DVL NAVIGATION", "0.4 m", "drift after 60 m · Doppler velocity log › ArduSub EKF")),
 "r3": ("split", ("capB_sonar", "capB_rcam", 58, 1.0), ("MULTIBEAM SONAR + LOW-LIGHT CAMERA", "", "130° · 12 m · sees through water the camera can't")),
 "t1": ("rviz", ("capB/rviz.mp4", 60, 2.0), ("THE HANDOFF", "", "drone flags at the water line › ROV dives below each one")),
 "t2": ("rviz", ("capB/rviz.mp4", 150, 2.0), ("OUR RECORDED RUN", "4 / 4", "waterline defects flagged, all inspected underwater")),
 "t3": ("still", ("rviz_3d_crop.png", 1.0, 1.10), ("ONE 3D MODEL", "4.54 M", "2.96 M lidar points above + 1.58 M sonar points below")),
 "k1": ("card", ("rviz_3d_crop.png",), kiro_card),
 "k2": ("dash", (0, 1.0), ("INSPECTOR DASHBOARD · BUILT WITH KIRO", "", "Megha's detection pipeline, end to end on our rosbag")),
 "k3": ("dash", (1, 1.0), ("DETECT › RETRACE › SEVERITY", "", "YOLOv8-seg on drone frames · lidar at each hit · rule-based severity")),
 "k4": ("dash", (2, 1.0), ("REPORT AGENT + PE SIGN-OFF", "", "every claim cites a detection id · a licensed engineer approves")),
 "k5": ("card", ("basin.png",), triage_card),
 "m1": ("card", ("canal.png",), lambda d: money_chart(d, False)),
 "m2": ("card", ("canal.png",), lambda d: money_chart(d, True)),
 "m3": ("card", ("under.png",), kit_chart),
 "w1": ("card", ("basin.png",), lambda d: timeline_card(d, "snbi")),
 "w2": ("card", ("basin.png",), lambda d: timeline_card(d, "money")),
 "w3": ("card", ("basin.png",), lambda d: timeline_card(d, "nh")),
 "c1": ("clip", ("capA_dchase", 126, 1.0, ["capB_rchase"]), ("NEXT", "10", "bridges in a pilot · every robot map checked against a diver")),
 "c2": ("still", ("lights.png", 1.0, 1.12), end_card),
 "title": ("clip", ("capB_rchase", 20, 0.8, None), title_card),
}


def overlay(k, spec):
    im = canvas(); d = ImageDraw.Draw(im)
    if callable(spec): spec(d)
    else: lower_third(d, *spec, im)
    if k not in ("title", "c2"): bug(d)
    p = B("ov", f"{k}.png"); im.save(p); return p


def ff(args):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + args, check=True)


ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "17", "-pix_fmt", "yuv420p", "-r", str(FPS)]
BASE = f"scale={W}:{HH}:force_original_aspect_ratio=increase,crop={W}:{HH},setsar=1"
GRADE = "eq=contrast=1.06:saturation=1.08,vignette=PI/5"
DASH = [P("dash", f"dash_{i}.mp4") for i in range(3)]


def ov_in(ov, slide):
    """overlay input + filter: lower thirds slide in from the left over 0.45 s; cards fade in."""
    return f"[ov]format=rgba,fade=t=in:st=0:d=0.35:alpha=1[ova]", ("overlay=x='if(lt(t,0.45),-400+400*(t/0.45)*(2-t/0.45),0)':y=0" if slide else "overlay=0:0")


def seg(k, dur):
    kind, a, spec = SH[k]; ov = overlay(k, spec); out = B("segs", f"{k}.mp4"); n = int(round(dur * FPS))
    slide = not callable(spec)
    fo, ovf = ov_in(ov, slide)
    if kind in ("still", "card"):
        img = os.path.join(ST, a[0]); z0, z1 = (a[1], a[2]) if kind == "still" else (1.0, 1.05)
        blur = ",gblur=sigma=22,eq=brightness=-0.10" if kind == "card" else ""
        fc = (f"[0:v]scale=3840:2160:force_original_aspect_ratio=increase,crop=3840:2160,zoompan=z='{z0}+({z1}-{z0})*on/{n}':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d={n}:s={W}x{HH}:fps={FPS}{blur},{GRADE}[b];"
              f"[1:v]copy[ov];{fo};[b][ova]{ovf}[v]")
        ff(["-loop", "1", "-i", img, "-loop", "1", "-i", ov, "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "clip":
        st, t0, sp, pips = a
        src_, t0_ = B("..", "build", f"{st}.mp4"), t0
        for mi0, mil in ((0, 60), (212, 24)):                                   # motion-interpolated ROV ranges (true 30 fps)
            mf = B("..", "build", f"{st}_mi_{mi0}.mp4")
            if st == "capB_rchase" and mi0 <= t0 and t0 + dur * sp <= mi0 + mil and os.path.exists(mf): src_, t0_ = mf, t0 - mi0
        ins = ["-ss", str(t0_), "-i", src_]; fc = f"[0:v]setpts=PTS/{sp},{BASE},{GRADE}[m];"; last = "m"
        for j, ps in enumerate(pips or []):
            ins += ["-ss", str(t0), "-i", B("..", "build", f"{ps}.mp4")]
            fc += f"[{j + 1}:v]setpts=PTS/{sp},scale=520:-2,pad=iw+6:ih+6:3:3:color=0x6ECDFF[p{j}];[{last}][p{j}]overlay=W-w-64:{110 + j * 320}[o{j}];"; last = f"o{j}"
        ins += ["-loop", "1", "-i", ov]
        fc += f"[{len(pips or []) + 1}:v]copy[ov];{fo};[{last}][ova]{ovf}[v]"
        ff(ins + ["-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "split":
        a1, a2, t0, sp = a
        fc = (f"[0:v]setpts=PTS/{sp},scale=940:-2,pad=960:{HH}:10:(oh-ih)/2-60:color=0x05090F[l];[1:v]setpts=PTS/{sp},scale=940:-2,pad=960:{HH}:10:(oh-ih)/2-60:color=0x05090F[r];"
              f"[l][r]hstack[m];[2:v]copy[ov];{fo};[m][ova]{ovf}[v]")
        ff(["-ss", str(t0), "-i", B("..", "build", f"{a1}.mp4"), "-ss", str(t0), "-i", B("..", "build", f"{a2}.mp4"), "-loop", "1", "-i", ov,
            "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "rviz":
        src, t0, sp = a
        fc = f"[0:v]setpts=PTS/{sp},crop=1416:797:468:140,{BASE}[m];[1:v]copy[ov];{fo};[m][ova]{ovf}[v]"
        ff(["-ss", str(t0), "-i", P(src), "-loop", "1", "-i", ov, "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    elif kind == "dash":
        i, sp = a
        fc = f"[0:v]setpts=PTS/{sp},{BASE}[m];[1:v]copy[ov];{fo};[m][ova]{ovf}[v]"
        ff(["-stream_loop", "-1", "-i", DASH[i], "-loop", "1", "-i", ov, "-filter_complex", fc, "-map", "[v]", "-frames:v", str(n)] + ENC + [out])
    return out


# segments: title over the open, then one per VO line; each runs XF longer so neighbours can cross-fade
starts = [("o1", 0.0)]
TS = json.load(open(B("timeline.json")))["title"]
for i, (k, t0, d) in enumerate(TL):
    if k == "o1": continue
    starts.append((k, t0 if k != "p1" else TS[0] + TS[1]))
    if k == "o2": starts.append(("title", TS[0]))
starts.sort(key=lambda x: x[1])
segs = []
for i, (k, s0) in enumerate(starts):
    e = starts[i + 1][1] if i + 1 < len(starts) else TOTAL
    segs.append((k, s0, seg(k, e - s0 + (XF if i + 1 < len(starts) else 0))))
    print(f"{k:6s} {s0:6.1f}s  {e - s0:5.2f}s", flush=True)
# xfade chain: act changes get a wipe, everything else a dissolve
ins, fc, last = [], "", "0:v"
for i, (k, s0, f) in enumerate(segs): ins += ["-i", f]
for i in range(1, len(segs)):
    k, s0, _ = segs[i]
    tr = "wipeleft" if k in ACT or k == "title" else ("fadeblack" if k == "c2" else "fade")
    fc += f"[{last}][{i}:v]xfade=transition={tr}:duration={XF}:offset={s0:.3f}[x{i}];"; last = f"x{i}"
ff(ins + ["-filter_complex", fc.rstrip(";"), "-map", f"[{last}]", "-t", f"{TOTAL:.3f}"] + ENC + [B("video_noaudio.mp4")])
print("VIDEO", f"{TOTAL:.1f}s", len(segs), "segments")
