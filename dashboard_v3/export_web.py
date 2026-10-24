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
# Crop both maps to the canal + bridge corridor in the canal frame (from
# route_dam: c along the canal, r across it) before downsampling. The far city
# LiDAR tail otherwise swamps the 3D view.
CROP_C = (-75.0, 40.0)   # along-canal extent (upstream approach .. downstream)
CROP_R = 22.0            # half-width across the canal (|r| < CROP_R)
CROP_Z = (-7.0, 14.0)    # vertical extent (below bed .. above the deck/towers)


def _canal_axes():
    """Return (C, R) unit vectors of the canal frame from route_dam, or None."""
    import numpy as np
    for cand in (Path.home() / "hackachusetts" / "sim",
                 Path.home() / "hackachusetts_ext" / "hackachusetts" / "sim"):
        if (cand / "route_dam.py").exists():
            sys.path.insert(0, str(cand))
            break
    try:
        import route_dam as p  # noqa: WPS433
        return np.asarray(p.C, float), np.asarray(p.R, float)
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"  WARN could not import route_dam ({exc}); skipping canal crop")
        return None


_ROUTE_DAM = None
_ROUTE_DAM_TRIED = False


def _route_dam():
    """Import and cache the route_dam module (geo frame + under_deck test)."""
    global _ROUTE_DAM, _ROUTE_DAM_TRIED
    if _ROUTE_DAM_TRIED:
        return _ROUTE_DAM
    _ROUTE_DAM_TRIED = True
    for cand in (Path.home() / "hackachusetts" / "sim",
                 Path.home() / "hackachusetts_ext" / "hackachusetts" / "sim"):
        if (cand / "route_dam.py").exists():
            sys.path.insert(0, str(cand))
            break
    try:
        import route_dam as p  # noqa: WPS433
        _ROUTE_DAM = p
    except Exception as exc:  # pragma: no cover
        print(f"  WARN could not import route_dam for geo frame ({exc})")
        _ROUTE_DAM = None
    return _ROUTE_DAM


def _to_latlon(geo, pts):
    """Convert a list of world [x,y,z] to [[lat, lon], ...] via route_dam.to_geo."""
    if geo is None:
        return []
    import numpy as np
    out = []
    for p in pts:
        lat, lon, _z = geo.to_geo(np.asarray(p, float))
        out.append([round(lat, 7), round(lon, 7)])
    return out


def crop_to_canal(points, axes):
    """Keep only points inside the canal/bridge corridor (canal-frame box)."""
    import numpy as np
    if axes is None:
        return points
    C, R = axes
    p = np.asarray(points, dtype=np.float64)[:, :3]
    c = p[:, 0] * C[0] + p[:, 1] * C[1]
    r = p[:, 0] * R[0] + p[:, 1] * R[1]
    mask = ((c > CROP_C[0]) & (c < CROP_C[1]) & (np.abs(r) < CROP_R)
            & (p[:, 2] > CROP_Z[0]) & (p[:, 2] < CROP_Z[1]))
    return p[mask]


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
    axes = _canal_axes()
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
        n_raw = len(pts)
        cropped = crop_to_canal(pts, axes)
        ds = voxel_downsample(cropped, voxel=voxel, max_points=max_points)
        (web_data / f"{key}.bin").write_bytes(ds.tobytes())
        samples.append(ds)
        mn = ds.min(axis=0).tolist()
        mx = ds.max(axis=0).tolist()
        meta[key] = {
            "file": f"{key}.bin",
            "count": int(len(ds)),
            "bounds": {"min": [round(v, 3) for v in mn], "max": [round(v, 3) for v in mx]},
            "source_count": int(n_raw),
        }
        print(f"  {key}: {n_raw:,} -> crop {len(cropped):,} -> {len(ds):,} pts (voxel {voxel} m)")

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


# status-string parsers (formats verified from the bag):
#   /drone/status: "GNSS 12 sats HDOP 1.21 RTK FIXED   |  ArduCopter AUTO  ARMED  item 1/37"
#   /rov/status:   "ArduSub GUIDED wall_E_submerged  depth 2.4 m  DVL alt 1.46 m lock True  nav err 0.09 m"
_RE_SATS = re.compile(r"GNSS\s+(\d+)\s+sats")
_RE_HDOP = re.compile(r"HDOP\s+([\d.]+)")
_RE_ITEM = re.compile(r"item\s+(\d+)/(\d+)")
_RE_DEPTH = re.compile(r"depth\s+([\-\d.]+)\s*m")
_RE_DVLALT = re.compile(r"DVL\s+alt\s+([\-\d.]+)\s*m")
_RE_LOCK = re.compile(r"lock\s+(True|False)")
_RE_NAVERR = re.compile(r"nav\s+err\s+([\-\d.]+)\s*m")


def _resample(series, n=200):
    """Downsample a [t, v] list to ~n evenly spaced points (keeps endpoints)."""
    if len(series) <= n:
        return [[round(t, 1), round(v, 3)] for t, v in series]
    step = len(series) / n
    out = []
    for k in range(n):
        t, v = series[int(k * step)]
        out.append([round(t, 1), round(v, 3)])
    t, v = series[-1]
    out.append([round(t, 1), round(v, 3)])
    return out


def decode_bag(bag_dir: Path):
    """Single pass over the bag for paths, ROV accuracy, cooperation, status and
    the per-vehicle time-series (GNSS sats, altitude, speed, depth, DVL alt,
    nav error) used by the dashboard charts and the big stats grid."""
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
            "/odom", "/rov/odom", "/rov/truth/odom",
            "/coop/tasks", "/rov/status", "/drone/status"]

    drone_path = rov_path = rov_truth_path = []
    rov_odom_last = rov_truth_last = None
    coop_events = []          # first time each marker text appears
    seen_coop = set()
    rov_status_samples = []   # a few status strings (kept for the UI)
    drone_item_max = 0
    t0 = None
    tend = None

    # time-series collectors: [t_rel, value]
    ts_alt, ts_speed = [], []        # drone altitude (z), ground speed
    ts_sats, ts_hdop = [], []        # drone GNSS satellites / HDOP
    ts_depth, ts_dvlalt = [], []     # ROV depth below water, DVL altitude above bed
    ts_naverr = []                   # ROV |DVL - truth| horizontal+vertical error
    # for drone speed and mission start/end of the under-deck shaded interval
    drone_odom_prev = None           # (t, [x,y,z])
    # ROV paired odom samples for a dense nav-error series
    rov_odom_buf = {}                # t_round -> est position
    rov_truth_buf = {}               # t_round -> truth position
    dvl_lock_true = 0
    dvl_lock_total = 0
    under_deck_t = []                # drone timestamps while under the deck
    sats_seen = []
    odom_hist = []

    # import the geo helper / under_deck test from route_dam
    geo = _route_dam()

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for sch, ch, m in reader.iter_messages(topics=want):
            t = m.log_time / 1e9
            if t0 is None:
                t0 = t
            tr = t - t0
            tend = t
            topic = ch.topic
            if topic == "/path":
                drone_path = _path_points(decode(ch, sch, m.data))
            elif topic == "/rov/path":
                rov_path = _path_points(decode(ch, sch, m.data))
            elif topic == "/rov/truth/path":
                rov_truth_path = _path_points(decode(ch, sch, m.data))
            elif topic == "/odom":
                od = decode(ch, sch, m.data)
                p = od.pose.pose.position
                pos = [p.x, p.y, p.z]
                ts_alt.append([tr, p.z])
                # ground speed from the pose itself over >= 1 s windows of bag time (the twist is a frame-to-frame
                # difference inside the renderer and spikes when a frame stalls)
                odom_hist.append((tr, pos))
                while len(odom_hist) > 2 and odom_hist[-1][0] - odom_hist[1][0] >= 1.0: odom_hist.pop(0)
                if odom_hist[-1][0] - odom_hist[0][0] >= 1.0:
                    (t0_, p0_), (t1_, p1_) = odom_hist[0], odom_hist[-1]
                    ts_speed.append([tr, math.dist(p0_, p1_) / (t1_ - t0_)])
                drone_odom_prev = (tr, pos)
                if geo and geo.under_deck(__import__("numpy").asarray(pos, float)):
                    under_deck_t.append(tr)
            elif topic == "/rov/odom":
                p = decode(ch, sch, m.data).pose.pose.position
                rov_odom_last = (tr, [p.x, p.y, p.z])
                rov_odom_buf[round(tr, 1)] = [p.x, p.y, p.z]
                ts_depth.append([tr, -p.z])  # depth below water line (z=0, Z up)
            elif topic == "/rov/truth/odom":
                p = decode(ch, sch, m.data).pose.pose.position
                rov_truth_last = (tr, [p.x, p.y, p.z])
                rov_truth_buf[round(tr, 1)] = [p.x, p.y, p.z]
            elif topic == "/coop/tasks":
                for mk in decode(ch, sch, m.data).markers:
                    txt = (mk.text or "").strip()
                    if txt and txt not in seen_coop:
                        seen_coop.add(txt)
                        coop_events.append({"t": round(tr, 1), "text": txt})
            elif topic == "/rov/status":
                txt = decode(ch, sch, m.data).data
                if len(rov_status_samples) < 4:
                    rov_status_samples.append({"t": round(tr, 1), "text": txt})
                md = _RE_DVLALT.search(txt)
                if md:
                    ts_dvlalt.append([tr, float(md.group(1))])
                ml = _RE_LOCK.search(txt)
                if ml:
                    dvl_lock_total += 1
                    if ml.group(1) == "True":
                        dvl_lock_true += 1
            elif topic == "/drone/status":
                txt = decode(ch, sch, m.data).data
                mobj = _RE_ITEM.search(txt)
                if mobj:
                    drone_item_max = max(drone_item_max, int(mobj.group(1)))
                ms = _RE_SATS.search(txt)
                if ms:
                    nsat = int(ms.group(1))
                    ts_sats.append([tr, nsat])
                    sats_seen.append(nsat)
                mh = _RE_HDOP.search(txt)
                if mh:
                    ts_hdop.append([tr, float(mh.group(1))])

    span = round((tend - t0), 1) if t0 is not None else 0.0

    # dense ROV nav-error series: pair est & truth on matched rounded times.
    for tk in sorted(set(rov_odom_buf) & set(rov_truth_buf)):
        ts_naverr.append([tk, math.dist(rov_odom_buf[tk], rov_truth_buf[tk])])
    ts_naverr.sort()

    # final + max nav error.
    nav_err = None
    if rov_odom_last and rov_truth_last:
        nav_err = round(math.dist(rov_odom_last[1], rov_truth_last[1]), 3)
    nav_err_max = round(max((v for _, v in ts_naverr), default=0.0), 3) if ts_naverr else None

    rov_distance = round(_polyline_length(rov_truth_path), 1) if rov_truth_path else None
    drone_distance = round(_polyline_length(drone_path), 1) if drone_path else None

    # drone flight time = first to last odom sample.
    flight_time = round(ts_alt[-1][0] - ts_alt[0][0], 1) if len(ts_alt) >= 2 else None
    # ROV dive time = first to last rov odom sample.
    dive_time = None
    if ts_depth:
        dive_time = round(ts_depth[-1][0] - ts_depth[0][0], 1)

    max_alt = round(max((v for _, v in ts_alt), default=0.0), 2) if ts_alt else None
    # exclude the first few near-zero speed samples; report moving mean + max.
    sp_vals = [v for _, v in ts_speed]
    mean_speed = round(sum(sp_vals) / len(sp_vals), 2) if sp_vals else None
    max_speed = round(max(sp_vals), 2) if sp_vals else None
    max_depth = round(max((v for _, v in ts_depth), default=0.0), 2) if ts_depth else None
    # time under the deck = count of distinct ~1 s buckets the drone was under it
    time_under_deck = round(len({round(x) for x in under_deck_t}), 1) if under_deck_t else 0.0
    under_deck_interval = ([round(min(under_deck_t), 1), round(max(under_deck_t), 1)]
                           if under_deck_t else None)
    min_sats = min(sats_seen) if sats_seen else None
    max_sats = max(sats_seen) if sats_seen else None
    dvl_lock_pct = round(100.0 * dvl_lock_true / dvl_lock_total, 1) if dvl_lock_total else None

    return {
        "span_s": span,
        "drone_path": drone_path,
        "rov_path": rov_path,
        "rov_truth_path": rov_truth_path,
        "nav_err_m": nav_err,
        "nav_err_max_m": nav_err_max,
        "rov_distance_m": rov_distance,
        "drone_distance_m": drone_distance,
        "coop_events": coop_events,
        "rov_status": rov_status_samples,
        "drone_waypoint_max": drone_item_max,
        # stats grid extras
        "flight_time_s": flight_time,
        "dive_time_s": dive_time,
        "max_alt_m": max_alt,
        "mean_speed_ms": mean_speed,
        "max_speed_ms": max_speed,
        "max_depth_m": max_depth,
        "time_under_deck_s": time_under_deck,
        "under_deck_interval": under_deck_interval,
        "min_sats": min_sats,
        "max_sats": max_sats,
        "dvl_lock_pct": dvl_lock_pct,
        # time-series (resampled) for the charts
        "series": {
            "drone_alt": _resample(ts_alt),
            "drone_speed": _resample(ts_speed),
            "drone_sats": _resample(ts_sats),
            "drone_hdop": _resample(ts_hdop),
            "rov_depth": _resample(ts_depth),
            "rov_dvl_alt": _resample(ts_dvlalt),
            "rov_nav_err": _resample(ts_naverr),
        },
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
# bag metadata (message / topic counts) + big stats grid
# --------------------------------------------------------------------------- #
def parse_bag_metadata(bag_dir: Path):
    """Read total messages / topics / per-topic counts from metadata.yaml.

    Avoids a PyYAML dependency: the file is simple enough to scan with regexes.
    """
    meta = bag_dir / "metadata.yaml"
    info = {"total_messages": None, "topic_count": None, "topics": {}, "duration_s": None}
    if not meta.exists():
        return info
    text = meta.read_text(encoding="utf-8", errors="replace")
    mm = re.search(r"message_count:\s*(\d+)", text)
    if mm:
        info["total_messages"] = int(mm.group(1))
    md = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if md:
        info["duration_s"] = round(int(md.group(1)) / 1e9, 1)
    # pair each topic name with the following message_count.
    names = re.findall(r"name:\s*(/\S+)", text)
    counts = re.findall(r"^\s*message_count:\s*(\d+)\s*$", text, re.MULTILINE)
    # the first message_count is the bag total; the rest are per-topic in order.
    per_topic = counts[1:] if len(counts) > len(names) else counts
    for name, c in zip(names, per_topic):
        info["topics"][name] = int(c)
    info["topic_count"] = len(info["topics"])
    return info


def build_stats(bag, meta, frames_manifest, cloud_meta, defects, dive, result):
    """The big stats grid -- every value measured from the bag / maps / result.

    Returns a list of {group, label, value, unit} rows the UI renders verbatim.
    """
    above = sum(1 for d in defects if not d["underwater"])
    below = sum(1 for d in defects if d["underwater"])
    fr = frames_manifest or {}

    def fcount(key):
        return (fr.get(key) or {}).get("count")

    rows = [
        # drone / above water
        ("Drone (above water)", "Flight time", bag.get("flight_time_s"), "s"),
        ("Drone (above water)", "Distance flown", bag.get("drone_distance_m"), "m"),
        ("Drone (above water)", "Max altitude", bag.get("max_alt_m"), "m"),
        ("Drone (above water)", "Mean speed", bag.get("mean_speed_ms"), "m/s"),
        ("Drone (above water)", "Max speed", bag.get("max_speed_ms"), "m/s"),
        ("Drone (above water)", "Time under the deck", bag.get("time_under_deck_s"), "s"),
        ("Drone (above water)", "GNSS satellites", _minmax(bag.get("min_sats"), bag.get("max_sats")), ""),
        ("Drone (above water)", "Waypoints flown", bag.get("drone_waypoint_max"), ""),
        # ROV / below water
        ("BlueROV2 (below water)", "Dive time", bag.get("dive_time_s"), "s"),
        ("BlueROV2 (below water)", "Distance travelled", bag.get("rov_distance_m"), "m"),
        ("BlueROV2 (below water)", "Max depth", bag.get("max_depth_m"), "m"),
        ("BlueROV2 (below water)", "Final nav error", bag.get("nav_err_m"), "m"),
        ("BlueROV2 (below water)", "Max nav error", bag.get("nav_err_max_m"), "m"),
        ("BlueROV2 (below water)", "DVL bottom lock", bag.get("dvl_lock_pct"), "%"),
        # maps / sensing
        ("Mapping & sensing", "LiDAR points (above)", cloud_meta.get("lidar", {}).get("source_count"), ""),
        ("Mapping & sensing", "Sonar points (below)", cloud_meta.get("sonar", {}).get("source_count"), ""),
        ("Mapping & sensing", "LiDAR map voxels", cloud_meta.get("lidar", {}).get("count"), ""),
        ("Mapping & sensing", "Sonar map voxels", cloud_meta.get("sonar", {}).get("count"), ""),
        ("Mapping & sensing", "Drone RGB frames", fcount("rgb"), ""),
        ("Mapping & sensing", "Thermal frames", fcount("thermal"), ""),
        ("Mapping & sensing", "ROV camera frames", fcount("rov_cam"), ""),
        ("Mapping & sensing", "Sonar frames", fcount("sonar"), ""),
        # findings / cooperation
        ("Findings & cooperation", "Model detections", len(result.get("detections", [])), ""),
        ("Findings & cooperation", "Ground-truth defects", len(defects), ""),
        ("Findings & cooperation", "  above water", above, ""),
        ("Findings & cooperation", "  below water", below, ""),
        ("Findings & cooperation", "Drone→ROV tasks", len(dive.get("done", [])), ""),
        # bag
        ("Recorded bag", "Mission duration", bag.get("span_s"), "s"),
        ("Recorded bag", "Total ROS messages", meta.get("total_messages"), ""),
        ("Recorded bag", "Topics recorded", meta.get("topic_count"), ""),
    ]
    return [{"group": g, "label": l, "value": v, "unit": u} for (g, l, v, u) in rows]


def _minmax(a, b):
    if a is None and b is None:
        return None
    if a == b:
        return a
    return f"{a}–{b}"


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
        "canal_axis": ([round(float(v), 4) for v in _canal_axes()[0]]
                       if _canal_axes() is not None else [1.0, 0.0]),
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

    # 7. GPS / geo bundle (Leaflet): paths + defects converted to lat/lon.
    geo = _route_dam()
    gps = None
    if geo is not None:
        import numpy as np
        et = getattr(geo, "_ET", (42.368614, -71.071864))
        center = [round(float(geo.LAT0), 7), round(float(geo.LON0), 7)]
        drone_ll = _to_latlon(geo, bag["drone_path"])
        rov_ll = _to_latlon(geo, bag["rov_path"])
        rov_truth_ll = _to_latlon(geo, bag["rov_truth_path"])
        defect_ll = []
        for d in defects:
            lat, lon, _z = geo.to_geo(np.asarray(d["pos"], float))
            defect_ll.append({"id": d["id"], "type": d["type"], "severity": d["severity"],
                              "underwater": d["underwater"],
                              "lat": round(lat, 7), "lon": round(lon, 7)})
        gps = {
            "center": center,
            "craigie": center,
            "dam_bridge": [round(float(et[0]), 7), round(float(et[1]), 7)],
            "drone_path": drone_ll,
            "rov_path": rov_ll,
            "rov_truth_path": rov_truth_ll,
            "defects": defect_ll,
            "drone_start": drone_ll[0] if drone_ll else None,
            "drone_end": drone_ll[-1] if drone_ll else None,
            "rov_start": rov_ll[0] if rov_ll else None,
            "rov_end": rov_ll[-1] if rov_ll else None,
        }
        (web_data / "gps.json").write_text(json.dumps(gps, indent=1), encoding="utf-8")
        print(f"gps: {len(drone_ll)} drone + {len(rov_ll)} ROV points, "
              f"{len(defect_ll)} defect pins (centre {center})")
    else:
        print("gps: skipped (route_dam unavailable)")

    # 8. time-series for the charts.
    series = dict(bag["series"])
    series["under_deck_interval"] = bag["under_deck_interval"]
    series["span_s"] = bag["span_s"]
    (web_data / "series.json").write_text(json.dumps(series, indent=1), encoding="utf-8")
    print("series: " + ", ".join(f"{k}×{len(v)}" for k, v in bag["series"].items()))

    # 9. sensor frames (before stats so the manifest feeds the stats grid).
    need_frames = args.frames or not (web_data / "frames.json").exists()
    if need_frames:
        print("frames:")
        frames_manifest = extract_frames(bag_dir, web_data)
    else:
        print("frames: kept existing (pass --frames to re-extract)")
        try:
            frames_manifest = json.loads((web_data / "frames.json").read_text(encoding="utf-8"))
        except Exception:
            frames_manifest = {}

    # 10. big stats grid (measured from bag + maps + frames + result).
    meta = parse_bag_metadata(bag_dir)
    stats = build_stats(bag, meta, frames_manifest, cloud_meta, defects, dive, result)
    (web_data / "stats.json").write_text(
        json.dumps({"mission_id": scene["mission_id"], "rows": stats}, indent=1),
        encoding="utf-8")
    print(f"stats: {len(stats)} rows · {meta.get('total_messages')} msgs · "
          f"{meta.get('topic_count')} topics")

    # 11. triage removed — statewide bridge ranking is no longer part of the
    #     dashboard. Plain NBI condition facts for the report come from
    #     scripts/kiro_report.py instead.

    print(f"done -> {web_data}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
