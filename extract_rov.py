#!/usr/bin/env python3
"""Extract the ROV streams of a drone+ROV rosbag2 mission into small files.

    python extract_rov.py --bag ../navfix/bridge-inspection/data/run_20261024_134808

Reads the .mcap once and writes <bag>/rov/:
  telemetry.csv     t, x,y,z (DVL/EKF, /rov/odom), tx,ty,tz (truth), drift_m
  status.csv        t, mode, area, depth_m, dvl_alt_m, dvl_lock, nav_err_m   (/rov/status)
  sonar_index.csv   t, seq, x,y,z                      (/rov/sonar/image, posed by /rov/odom)
  camera_index.csv  t, seq, x,y,z, visibility          (/rov/camera; RMS contrast 0..1)
  coop_tasks.csv    t, defect_id, text                 (/coop/tasks, de-duplicated)
  frames/           sonar + camera stills nearest each underwater GT defect

The pipeline then runs from these files with the standard library only, the
same way the drone side runs from inspection_frames/. Needs: mcap,
mcap-ros2-support, zstandard, numpy, opencv-python (or Pillow) for the stills.
"""
import argparse
import bisect
import csv
import json
import math
import os
import re
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pipeline import rosbag as rb  # noqa: E402

STATUS_RE = re.compile(
    r"^(?P<vehicle>\S+)\s+(?P<mode>\S+)\s+(?P<area>.+?)\s+depth\s+(?P<depth>-?[\d.]+)\s*m"
    r"\s+DVL alt\s+(?P<alt>-?[\d.]+)\s*m\s+lock\s+(?P<lock>\w+)\s+nav err\s+(?P<err>-?[\d.]+)\s*m")
COOP_RE = re.compile(r"\b(D\d+)\b")


def _image_array(msg):
    import numpy as np
    ch = 3 if msg.encoding in ("rgb8", "bgr8") else 1
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, ch)
    return arr[:, :, ::-1] if msg.encoding == "rgb8" else arr   # -> BGR for cv2


def _visibility(arr) -> float:
    """RMS contrast of the luminance, 0..1. Turbid water flattens contrast, so
    this is the clarity metric the camera-confirm gate uses."""
    import numpy as np
    g = arr.mean(axis=2) if arr.ndim == 3 else arr
    return round(float(np.std(g.astype(np.float32) / 255.0)), 4)


def _save_png(path: Path, arr) -> None:
    try:
        import cv2  # type: ignore
        cv2.imwrite(str(path), arr)
    except ImportError:
        from PIL import Image  # type: ignore
        Image.fromarray(arr[:, :, ::-1] if arr.ndim == 3 else arr).save(path)


class _Timeline:
    def __init__(self):
        self.t, self.p = [], []

    def add(self, t, p):
        self.t.append(t)
        self.p.append(p)

    def at(self, t):
        if not self.t:
            return (0.0, 0.0, 0.0)
        i = bisect.bisect_left(self.t, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(self.t)), key=lambda k: abs(self.t[k] - t))
        return self.p[j]


def main() -> int:
    ap = argparse.ArgumentParser(description="Extract ROV streams from a drone+ROV bag")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default=None, help="output dir (default <bag>/rov)")
    ap.add_argument("--visibility-samples", default=None,
                    help="also save camera frames at the 5/25/50/75/95th visibility percentile here")
    args = ap.parse_args()

    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    bag = Path(args.bag)
    out = Path(args.out) if args.out else bag / "rov"
    (out / "frames").mkdir(parents=True, exist_ok=True)
    mcap_path = rb._find_mcap(str(bag))
    T = rb.ROV_TOPICS

    # Underwater GT defects: we keep the sonar/camera still nearest each one.
    gt_path = bag / "defects_gt.json"
    gt = json.loads(gt_path.read_text(encoding="utf-8-sig")).get("defects", []) if gt_path.exists() else []
    under = [d for d in gt if rb.gt_platform(d.get("position")) == "rov"]

    ekf, truth, yaw = _Timeline(), _Timeline(), _Timeline()
    status_rows, coop_rows, seen_coop, unparsed = [], [], set(), []
    seq_of = {}                # per-topic running index (the bag leaves message.sequence = 0)
    sonar_t, cam = [], []      # (t, seq) / (t, seq, visibility)
    best = {}                  # (defect_id, kind) -> (dist, seq, t)
    dec = DecoderFactory()
    decoders = {}

    def decode(ch, schema, msg):
        if ch.topic not in decoders:
            decoders[ch.topic] = dec.decoder_for(ch.message_encoding, schema)
        return decoders[ch.topic](msg.data)

    print(f"pass 1/2: poses, status, indexes from {mcap_path.name} ...", flush=True)
    with open(mcap_path, "rb") as f:
        for schema, ch, msg in make_reader(f).iter_messages(
                topics=[T["odom"], T["truth"], T["status"], T["sonar"], T["camera"], "/coop/tasks"]):
            t = msg.log_time / 1e9
            seq = seq_of.get(ch.topic, 0)
            seq_of[ch.topic] = seq + 1
            if ch.topic in (T["odom"], T["truth"]):
                pose = decode(ch, schema, msg).pose.pose
                p = pose.position
                (ekf if ch.topic == T["odom"] else truth).add(t, (p.x, p.y, p.z))
                if ch.topic == T["odom"]:
                    q = pose.orientation
                    yaw.add(t, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            elif ch.topic == T["status"]:
                text = decode(ch, schema, msg).data.strip()
                m = STATUS_RE.match(text)
                if m:
                    area = m["area"].strip()
                    task = re.match(r"task (D\d+)", area)   # "task D03: below efflorescence"
                    area = f"coop_task_{task[1]}" if task else area
                    status_rows.append([f"{t:.3f}", m["mode"], area, m["depth"], m["alt"],
                                        m["lock"] == "True", m["err"]])
                else:
                    unparsed.append(text)
            elif ch.topic == "/coop/tasks":
                for mk in decode(ch, schema, msg).markers:
                    did = COOP_RE.search(mk.text or "")
                    if did and (did[1], mk.text) not in seen_coop:
                        seen_coop.add((did[1], mk.text))
                        coop_rows.append([f"{t:.3f}", did[1], mk.text])
            elif ch.topic == T["sonar"]:
                sonar_t.append((t, seq))
            elif ch.topic == T["camera"]:
                arr = _image_array(decode(ch, schema, msg))
                cam.append((t, seq, _visibility(arr)))

    def nearest(kind, rows):
        for d in under:
            for row in rows:
                pos = ekf.at(row[0])
                dist = math.dist(pos, d["position"])
                key = (d["id"], kind)
                if key not in best or dist < best[key][0]:
                    best[key] = (dist, row[1], row[0])

    nearest("sonar", sonar_t)
    nearest("camera", cam)

    with (out / "telemetry.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "z", "tx", "ty", "tz", "drift_m"])
        for t, p in zip(ekf.t, ekf.p):
            q = truth.at(t)
            w.writerow([f"{t:.3f}", *(f"{v:.3f}" for v in p), *(f"{v:.3f}" for v in q),
                        f"{math.dist(p, q):.3f}"])
    with (out / "status.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "mode", "area", "depth_m", "dvl_alt_m", "dvl_lock", "nav_err_m"])
        w.writerows(status_rows)
    with (out / "sonar_index.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "seq", "x", "y", "z", "yaw"])
        for t, seq in sonar_t:
            w.writerow([f"{t:.3f}", seq, *(f"{v:.3f}" for v in ekf.at(t)), f"{yaw.at(t):.4f}"])
    with (out / "camera_index.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "seq", "x", "y", "z", "visibility"])
        for t, seq, vis in cam:
            w.writerow([f"{t:.3f}", seq, *(f"{v:.3f}" for v in ekf.at(t)), vis])
    with (out / "coop_tasks.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "defect_id", "text"])
        w.writerows(coop_rows)

    # pass 2: write the stills nearest each underwater GT defect (+ optional
    # visibility samples, used once to pick ROV_MIN_VISIBILITY by eye)
    wanted = {(k, v[1]): did for (did, k), v in best.items()}
    samples = {}
    if args.visibility_samples:
        by_vis = sorted(cam, key=lambda c: c[2])
        for q in (0.05, 0.25, 0.5, 0.75, 0.95):
            t_, seq_, v_ = by_vis[min(int(q * len(by_vis)), len(by_vis) - 1)]
            samples[seq_] = v_
    print(f"pass 2/2: writing {len(wanted)} stills ...", flush=True)
    counter = {}
    with open(mcap_path, "rb") as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=[T["sonar"], T["camera"]]):
            kind = "sonar" if ch.topic == T["sonar"] else "camera"
            seq = counter.get(kind, 0)
            counter[kind] = seq + 1
            if (kind, seq) in wanted:
                _save_png(out / "frames" / f"{kind}_{seq:05d}.png", _image_array(decode(ch, schema, msg)))
            if kind == "camera" and seq in samples:
                Path(args.visibility_samples).mkdir(parents=True, exist_ok=True)
                _save_png(Path(args.visibility_samples) / f"vis_{samples[seq]:.4f}_cam{seq:05d}.png",
                          _image_array(decode(ch, schema, msg)))
    if unparsed:
        print(f"  {len(unparsed)} /rov/status lines did not parse, e.g.: {unparsed[:3]}")

    vis = sorted(v for _, _, v in cam)
    print(f"wrote {out}: {len(ekf.t)} odom, {len(status_rows)} status, {len(sonar_t)} sonar, "
          f"{len(cam)} camera (visibility p10/p50/p90 = "
          f"{vis[len(vis) // 10] if vis else 0}/{vis[len(vis) // 2] if vis else 0}/"
          f"{vis[9 * len(vis) // 10] if vis else 0}), {len(coop_rows)} coop tasks")
    for (did, kind), (dist, seq, _) in sorted(best.items()):
        print(f"  {did} nearest {kind:<6} seq {seq:>5} at {dist:.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
