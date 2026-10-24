#!/usr/bin/env python3
"""Build the web dashboard data bundle for the two-robot (drone + ROV) mission.

This is the ONE command that turns a recorded drone+ROV bag into everything the
browser dashboard needs, under ``web/data/``:

  scene.json         3D scene description (cloud metadata, paths, water line,
                     ground-truth defects, drone->ROV cooperation dive lines,
                     scene bounds/centre so the viewer can auto-frame the canal)
  lidar.bin          above-water LiDAR map, voxel-downsampled, Float32 x,y,z
  sonar.bin          below-water sonar map,  voxel-downsampled, Float32 x,y,z
  kpis.json          headline numbers (all measured from the data)
  coop.json          cooperation timeline (drone flags -> ROV dives / done)
  result.json        the pipeline detection result (copied from --result)
  frames.json        per-stream sensor-frame manifest (rgb/thermal/rov_cam/sonar)
  <stream>/NNNN.jpg   ~60 evenly spaced stills extracted from each sensor stream
  triage.json        statewide NBI underwater-risk ranking (top N + summary)

Every number shown on the dashboard is derived here from the bag, the maps, the
ground-truth defect list, the cooperation log and the pipeline result -- nothing
is hard-coded in the UI.

Needs: numpy, mcap, mcap-ros2-support, Pillow (all in the project venv).
"""
import argparse
import json
import math
import os
import re
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

WATER_Z = 0.0  # world frame: water line is z = 0 (Z up, metres)

# Sensor streams to extract stills from: topic -> (folder, label, n_frames).
STREAMS = [
    ("/camera/rgb", "rgb", "Drone RGB", 60),
    ("/camera/thermal", "thermal", "Thermal (LWIR, simulated)", 60),
    ("/rov/camera", "rov_cam", "ROV camera (turbid)", 60),
    ("/rov/sonar/image", "sonar", "ROV sonar (multibeam fan)", 60),
]


# --------------------------------------------------------------------------- #
# point clouds
# --------------------------------------------------------------------------- #
def voxel_downsample(points, voxel, max_points):
    """Keep one representative point per voxel cell, then cap the total."""
    import numpy as np
    p = np.asarray(points, dtype=np.float64)[:, :3]
    keys = np.floor(p / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    out = p[np.sort(idx)]
    if len(out) > max_points:
        step = len(out) // max_points + 1
        out = out[::step]
    return out.astype(np.float32)


def export_clouds(map_dir, web_data, max_points):
    """Downsample the above-water LiDAR and below-water sonar maps.

    Returns (meta, frame) where meta is per-cloud metadata and frame is a robust
    (2nd-98th percentile) combined box so the viewer auto-frames the canal body
    rather than the far LiDAR tail.
    """
    import numpy as np
    meta = {}
    samples = []  # downsampled points pooled for the robust frame box
    sources = [
        ("lidar", "drone_lidar_world.npy", 0.12),
        ("sonar", "rov_sonar_world.npy", 0.10),
    ]
    for key, fname, voxel in sources:
        npy = map_dir / fname
        if not npy.exists():
            print(f"  WARN no {fname}; skipping {key} cloud")
            continue
        pts = np.load(str(npy))[:, :3]
        ds = voxel_downsample(pts, voxel=voxel, max_points=max_points)
        (web_data / f"{key}.bin").write_bytes(ds.tobytes())
        samples.append(ds)
        mn = ds.min(axis=0).tolist()
        mx = ds.max(axis=0).tolist()
        meta[key] = {
            "file": f"{key}.bin",
            "count": int(len(ds)),
            "bounds": {"min": [round(v, 3) for v in mn], "max": [round(v, 3) for v in mx]},
            "source_count": int(len(pts)),
        }
        print(f"  {key}: {len(pts):,} -> {len(ds):,} pts (voxel {voxel} m)")

    frame = None
    if samples:
        allp = np.concatenate(samples, axis=0)
        lo = np.percentile(allp, 2, axis=0)
        hi = np.percentile(allp, 98, axis=0)
        center = [round(float((lo[i] + hi[i]) / 2), 3) for i in range(3)]
        span = [round(float(hi[i] - lo[i]), 3) for i in range(3)]
        frame = {"min": [round(float(v), 3) for v in lo],
                 "max": [round(float(v), 3) for v in hi],
                 "center": center, "span": span}
    return meta, frame


# --------------------------------------------------------------------------- #
# bag decode: paths, ROV navigation accuracy, cooperation, status
# --------------------------------------------------------------------------- #
def _find_mcap(bag_dir: Path) -> Path:
    if bag_dir.is_file() and bag_dir.suffix == ".mcap":
        return bag_dir
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if not mcaps:
        raise FileNotFoundError(f"no .mcap in {bag_dir}")
    return mcaps[0]


def _path_points(path_msg):
    return [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in path_msg.poses]


def _polyline_length(pts):
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        total += math.dist(a, b)
    return total


def decode_bag(bag_dir: Path):
    """Single pass over the bag for paths, ROV accuracy and cooperation events."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    mcap_path = _find_mcap(bag_dir)
    dec = DecoderFactory()
    decoders = {}

    def decode(ch, sch, data):
        d = decoders.get(ch.topic)
        if d is None:
            d = dec.decoder_for(ch.message_encoding, sch)
            decoders[ch.topic] = d
        return d(data)

    want = ["/path", "/rov/path", "/rov/truth/path",
            "/rov/odom", "/rov/truth/odom",
            "/coop/tasks", "/rov/status", "/drone/status"]

    drone_path = rov_path = rov_truth_path = []
    rov_odom_last = rov_truth_last = None
    coop_events = []          # (t_rel, text) first time each marker text appears
    seen_coop = set()
    rov_status_samples = []   # (t_rel, text) a few status strings
    drone_item_max = 0
    t0 = None

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for sch, ch, m in reader.iter_messages(topics=want):
            t = m.log_time / 1e9
            if t0 is None:
                t0 = t
            tr = t - t0
            topic = ch.topic
            if topic == "/path":
                drone_path = _path_points(decode(ch, sch, m.data))
            elif topic == "/rov/path":
                rov_path = _path_points(decode(ch, sch, m.data))
            elif topic == "/rov/truth/path":
                rov_truth_path = _path_points(decode(ch, sch, m.data))
            elif topic == "/rov/odom":
                p = decode(ch, sch, m.data).pose.pose.position
                rov_odom_last = (tr, [p.x, p.y, p.z])
            elif topic == "/rov/truth/odom":
                p = decode(ch, sch, m.data).pose.pose.position
                rov_truth_last = (tr, [p.x, p.y, p.z])
            elif topic == "/coop/tasks":
                for mk in decode(ch, sch, m.data).markers:
                    txt = (mk.text or "").strip()
                    if txt and txt not in seen_coop:
                        seen_coop.add(txt)
                        coop_events.append({"t": round(tr, 1), "text": txt})
            elif topic == "/rov/status":
                if len(rov_status_samples) < 4:
                    rov_status_samples.append({"t": round(tr, 1),
                                               "text": decode(ch, sch, m.data).data})
            elif topic == "/drone/status":
                txt = decode(ch, sch, m.data).data
                mobj = re.search(r"item\s+(\d+)/(\d+)", txt)
                if mobj:
                    drone_item_max = max(drone_item_max, int(mobj.group(1)))

    span = round((m.log_time / 1e9) - t0, 1) if t0 is not None else 0.0

    # ROV navigation accuracy: DVL dead-reckoned estimate vs. ground truth.
    nav_err = None
    if rov_odom_last and rov_truth_last:
        nav_err = round(math.dist(rov_odom_last[1], rov_truth_last[1]), 3)
    rov_distance = round(_polyline_length(rov_truth_path), 1) if rov_truth_path else None

    return {
        "span_s": span,
        "drone_path": drone_path,
        "rov_path": rov_path,
        "rov_truth_path": rov_truth_path,
        "nav_err_m": nav_err,
        "rov_distance_m": rov_distance,
        "coop_events": coop_events,
        "rov_status": rov_status_samples,
        "drone_waypoint_max": drone_item_max,
    }


# --------------------------------------------------------------------------- #
# sensor-frame extraction (~60 evenly spaced stills per stream)
# --------------------------------------------------------------------------- #
def _image_to_array(np, msg):
    """sensor_msgs/Image -> HxWx(3|1) uint8 ndarray (rgb8 / mono8 / bgr8)."""
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = (msg.encoding or "rgb8").lower()
    ch = 1 if enc in ("mono8", "8uc1") else 3
    arr = data.reshape(msg.height, msg.width, ch)
    if enc == "bgr8":
        arr = arr[:, :, ::-1]
    if ch == 1:
        arr = np.repeat(arr, 3, axis=2)
    return arr


def extract_frames(bag_dir: Path, web_data: Path):
    """Extract ~N evenly spaced stills from each sensor stream to <stream>/NNNN.jpg.

    Done in a single streaming pass per topic: first count messages, then pick
    evenly spaced indices and re-iterate writing only those (keeps memory low).
    """
    import numpy as np
    from PIL import Image
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    mcap_path = _find_mcap(bag_dir)
    dec = DecoderFactory()
    manifest = {}

    # 1) count messages per topic (cheap metadata pass).
    counts = {topic: 0 for topic, *_ in STREAMS}
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _, ch, _ in reader.iter_messages(topics=list(counts)):
            counts[ch.topic] += 1

    # 2) decode & write only the chosen indices for each topic.
    for topic, folder, label, n in STREAMS:
        total = counts.get(topic, 0)
        out_dir = web_data / folder
        out_dir.mkdir(parents=True, exist_ok=True)
        for old in out_dir.glob("*.jpg"):
            old.unlink()
        if total == 0:
            manifest[folder] = {"label": label, "topic": topic, "count": 0, "frames": []}
            print(f"  {folder}: no frames on {topic}")
            continue
        take = min(n, total)
        want_idx = {round(k * (total - 1) / max(take - 1, 1)) for k in range(take)}
        decoder = None
        written, idx_seen = [], -1
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            for sch, ch, m in reader.iter_messages(topics=[topic]):
                idx_seen += 1
                if idx_seen not in want_idx:
                    continue
                if decoder is None:
                    decoder = dec.decoder_for(ch.message_encoding, sch)
                msg = decoder(m.data)
                arr = _image_to_array(np, msg)
                name = f"{len(written):04d}.jpg"
                Image.fromarray(arr).save(out_dir / name, quality=82)
                written.append({"name": name, "idx": idx_seen})
        manifest[folder] = {"label": label, "topic": topic, "count": total,
                            "frames": written}
        print(f"  {folder}: {len(written)} stills from {total} ({label})")

    (web_data / "frames.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    return manifest


# --------------------------------------------------------------------------- #
# ground-truth defects + cooperation dive lines
# --------------------------------------------------------------------------- #
def build_defects(bag_dir: Path):
    gt = json.loads((bag_dir / "defects_gt.json").read_text(encoding="utf-8-sig"))
    defects = []
    for d in gt.get("defects", []):
        pos = d.get("position", [0, 0, 0])
        defects.append({
            "id": d["id"], "type": d["type"], "severity": int(d.get("severity", 2)),
            "element": d.get("element", ""), "pos": [round(v, 3) for v in pos],
            "underwater": pos[2] < WATER_Z,
        })
    return gt.get("bridge", ""), defects


def build_dive_lines(bag_dir: Path, defects):
    """Drone-flagged waterline defect -> ROV dive point directly below it.

    Parsed from coop_log.txt (the authoritative record of which defects the
    drone flagged). The dive point is the wall base below the flag.
    """
    log = (bag_dir / "coop_log.txt").read_text(encoding="utf-8")
    by_id = {d["id"]: d for d in defects}
    flagged, done = [], set()
    for line in log.splitlines():
        mflag = re.match(r"COOP drone flagged (D\d+)", line)
        mdone = re.search(r"TASK DONE task (D\d+)", line)
        if mflag:
            flagged.append(mflag.group(1))
        if mdone:
            done.add(mdone.group(1))
    lines = []
    for did in flagged:
        d = by_id.get(did)
        if not d:
            continue
        top = list(d["pos"])
        bottom = [top[0], top[1], -3.5]  # ROV dives to the wall base below the flag
        lines.append({"id": did, "type": d["type"], "top": top, "bottom": bottom,
                      "done": did in done})
    return {"flagged": flagged, "done": sorted(done), "lines": lines}


# --------------------------------------------------------------------------- #
# KPIs + scene bounds
# --------------------------------------------------------------------------- #
def scene_bounds(cloud_meta):
    """Union of all cloud bounds -> {min,max,center,span} for camera auto-frame."""
    mins = [1e9, 1e9, 1e9]
    maxs = [-1e9, -1e9, -1e9]
    for m in cloud_meta.values():
        bmin, bmax = m["bounds"]["min"], m["bounds"]["max"]
        for i in range(3):
            mins[i] = min(mins[i], bmin[i])
            maxs[i] = max(maxs[i], bmax[i])
    center = [round((mins[i] + maxs[i]) / 2, 3) for i in range(3)]
    span = [round(maxs[i] - mins[i], 3) for i in range(3)]
    return {"min": [round(v, 3) for v in mins], "max": [round(v, 3) for v in maxs],
            "center": center, "span": span}


def build_kpis(result, defects, dive, bag, cloud_meta):
    above = [d for d in defects if not d["underwater"]]
    below = [d for d in defects if d["underwater"]]
    m = result.get("metrics", {})
    return {
        "model_defects": m.get("defect_count", len(result.get("detections", []))),
        "model_severe": m.get("severe", 0),
        "model_poor": m.get("poor", 0),
        "model_fair": m.get("fair", 0),
        "gt_total": len(defects),
        "gt_above": len(above),
        "gt_below": len(below),
        "flagged": len(dive["flagged"]),
        "rov_inspected": len(dive["done"]),
        "nav_err_m": bag["nav_err_m"],
        "rov_distance_m": bag["rov_distance_m"],
        "lidar_points": cloud_meta.get("lidar", {}).get("source_count", 0),
        "sonar_points": cloud_meta.get("sonar", {}).get("source_count", 0),
        "mission_s": bag["span_s"],
        "drone_waypoints": bag["drone_waypoint_max"],
    }


# --------------------------------------------------------------------------- #
# triage
# --------------------------------------------------------------------------- #
def export_triage(web_data, triage_py, triage_venv, top_n):
    """Run the statewide NBI triage and keep the top N + summary counts."""
    import subprocess
    import tempfile
    if not Path(triage_py).exists():
        print(f"  WARN triage script not found at {triage_py}; skipping")
        return
    tmp = Path(tempfile.gettempdir()) / "detectfix_triage.json"
    proc = subprocess.run([triage_venv, triage_py, "--json", str(tmp), "--summary"],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"  WARN triage failed: {proc.stderr[:200]}")
        return
    ranking = json.loads(tmp.read_text(encoding="utf-8"))
    summary = _parse_triage_summary(proc.stdout)
    out = {"summary": summary, "top": ranking[:top_n]}
    (web_data / "triage.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"  triage: {summary.get('total','?')} bridges, top {len(out['top'])} kept")


def _parse_triage_summary(text):
    """Pull the headline counts out of the --summary stdout block."""
    def grab(pat):
        m = re.search(pat, text)
        return int(m.group(1).replace(",", "")) if m else None
    return {
        "total": grab(r"Bridges total\s*:\s*([\d,]+)"),
        "over_water": grab(r"Over water.*?:\s*([\d,]+)"),
        "scour_critical": grab(r"Scour critical.*?:\s*([\d,]+)"),
        "poor": grab(r"Poor condition \(BRIDGE_CONDITION[^:]*:\s*([\d,]+)"),
        "underwater_required": grab(r"Underwater inspection required\s*:\s*([\d,]+)"),
        "underwater_overdue": grab(r"\.\.\.overdue\s*:\s*([\d,]+)"),
    }


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description="Build the dashboard data bundle")
    ap.add_argument("--bag", required=True, help="drone+ROV bag directory")
    ap.add_argument("--result", default="out/bag2_model.json",
                    help="pipeline detection result JSON")
    ap.add_argument("--max-points", type=int, default=250000,
                    help="cap per cloud after voxel downsample")
    ap.add_argument("--frames", action="store_true",
                    help="(re-)extract sensor stills (slow; skips if omitted and dirs exist)")
    ap.add_argument("--no-triage", action="store_true")
    ap.add_argument("--triage-py",
                    default=str(Path.home() / "hackachusetts/ai/bridge_triage.py"))
    ap.add_argument("--triage-venv",
                    default=str(Path.home() / "hackachusetts/.venv/bin/python"))
    ap.add_argument("--triage-top", type=int, default=10)
    args = ap.parse_args()

    bag_dir = Path(args.bag)
    web_data = HERE / "web" / "data"
    web_data.mkdir(parents=True, exist_ok=True)

    print(f"exporting dashboard bundle for {bag_dir.name} ...")

    # 1. pipeline result
    result = json.loads(Path(args.result).read_text(encoding="utf-8"))
    (web_data / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    # 2. point clouds
    print("clouds:")
    cloud_meta, frame = export_clouds(bag_dir / "map", web_data, args.max_points)

    # 3. bag decode (paths, ROV accuracy, cooperation)
    print("bag decode:")
    bag = decode_bag(bag_dir)
    print(f"  mission {bag['span_s']} s · nav err {bag['nav_err_m']} m · "
          f"ROV travelled {bag['rov_distance_m']} m · {len(bag['coop_events'])} coop events")

    # 4. defects + dive lines
    bridge, defects = build_defects(bag_dir)
    dive = build_dive_lines(bag_dir, defects)

    # 5. KPIs + scene
    kpis = build_kpis(result, defects, dive, bag, cloud_meta)
    (web_data / "kpis.json").write_text(json.dumps(kpis, indent=1), encoding="utf-8")

    scene = {
        "bridge": bridge,
        "mission_id": result.get("mission_id", bag_dir.name),
        "water_z": WATER_Z,
        "clouds": cloud_meta,
        "bounds": scene_bounds(cloud_meta) if cloud_meta else None,
        "frame": frame,
        "drone_path": [[round(c, 2) for c in p] for p in bag["drone_path"]],
        "rov_path": [[round(c, 2) for c in p] for p in bag["rov_path"]],
        "rov_truth_path": [[round(c, 2) for c in p] for p in bag["rov_truth_path"]],
        "defects": defects,
        "dive_lines": dive["lines"],
        "detections": [
            {"id": d.get("detection_id"), "class": d.get("class"),
             "severity": d.get("severity"), "pos": d.get("world_pos")}
            for d in result.get("detections", []) if d.get("world_pos")
        ],
    }
    (web_data / "scene.json").write_text(json.dumps(scene, indent=1), encoding="utf-8")
    print(f"scene: {len(scene['drone_path'])} drone pts, {len(scene['rov_path'])} ROV pts, "
          f"{len(defects)} defects, {len(dive['lines'])} dive lines")

    # 6. cooperation timeline
    coop = {"span_s": bag["span_s"], "events": bag["coop_events"],
            "rov_status": bag["rov_status"], "log": dive}
    (web_data / "coop.json").write_text(json.dumps(coop, indent=1), encoding="utf-8")

    # 7. sensor frames
    need_frames = args.frames or not (web_data / "frames.json").exists()
    if need_frames:
        print("frames:")
        extract_frames(bag_dir, web_data)
    else:
        print("frames: kept existing (pass --frames to re-extract)")

    # 8. triage
    if not args.no_triage:
        print("triage:")
        export_triage(web_data, args.triage_py, args.triage_venv, args.triage_top)

    print(f"done -> {web_data}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
