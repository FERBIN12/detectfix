"""capture JPEG streams (timestamped in cap/frames.csv) -> constant 30 fps videos on a shared clock: build/<stream>.mp4"""
import csv, os, subprocess, collections
H = os.path.dirname(os.path.abspath(__file__)); os.makedirs(os.path.join(H, "build"), exist_ok=True)
rows = list(csv.DictReader(open(os.path.join(H, "cap", "frames.csv"))))
T0 = min(float(r["t"]) for r in rows); S = collections.defaultdict(list)
for r in rows: S[r["stream"]].append((float(r["t"]) - T0, int(r["frame"])))
for st, fr in S.items():
    lst = os.path.join(H, "build", f"{st}.ffconcat")
    with open(lst, "w") as f:
        f.write("ffconcat version 1.0\n")
        f.write(f"file '{H}/cap/{st}/{fr[0][1]:06d}.jpg'\nduration {fr[0][0]:.3f}\n")       # hold the first frame from t=0
        for i, (t, n) in enumerate(fr):
            d = (fr[i + 1][0] - t) if i + 1 < len(fr) else 0.5
            f.write(f"file '{H}/cap/{st}/{n:06d}.jpg'\nduration {d:.3f}\n")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-vf", "fps=30,scale=trunc(iw/2)*2:trunc(ih/2)*2,format=yuv420p",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", os.path.join(H, "build", f"{st}.mp4")], check=True)
    print(st, len(fr), "frames", f"{fr[-1][0]:.0f}s")
