"""ROV side of a drone+ROV mission, read from the <bag>/rov/ extract.

`extract_rov.py` turns the .mcap's ROV topics into small CSVs once; this module
reads them with the standard library only, the same way the drone side runs
from inspection_frames/. It provides:

  - sonar + rov_camera Captures posed by the ROV's own DVL/EKF estimate
    (/rov/odom), so retrace and camera-confirm use what the ROV believed,
  - GT-sourced ROV detections for underwater defects (labelled as ground truth;
    no sonar model weights exist for this sim),
  - the clarity gate: a camera frame confirms a sonar hit only if its RMS
    contrast is at least ROV_MIN_VISIBILITY.

Code enforces, models explain: every threshold here is a named constant.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Optional

from .indexing import Capture, TimePoseIndex
from .schema import Detection, Pose, make_detection

# Camera-confirm clarity gate: minimum RMS luminance contrast (0..1) for an ROV
# camera frame to count as a visual confirmation of a sonar detection.
# Picked by eye from run_20261024_134808 (extract_rov.py --visibility-samples):
# frames at 0.012 and 0.020 are green murk with no wall texture; frames at
# 0.036, 0.045 and 0.139 show the block joints / rock surface clearly. The gate
# sits between the two groups. Median over the mission is 0.036.
ROV_MIN_VISIBILITY = 0.03

# A GT defect is attached to the nearest sonar ping only if the ROV was within
# this range of it (the multibeam fan reaches 12 m per the bag README).
SONAR_RANGE_M = 12.0


def rov_dir(bag_dir: str) -> Path:
    return Path(bag_dir) / "rov"


def has_rov(bag_dir: str) -> bool:
    return (rov_dir(bag_dir) / "sonar_index.csv").exists()


def _rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8-sig", newline="") as fh:
        return list(csv.DictReader(fh))


def _f(v, default=0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load_captures(bag_dir: str) -> list[Capture]:
    """Sonar and ROV-camera captures from the extract, posed by /rov/odom."""
    d = rov_dir(bag_dir)
    caps: list[Capture] = []
    for r in _rows(d / "sonar_index.csv"):
        caps.append(Capture("sonar", _f(r["t"]), f"sonar:{r['seq']}",
                            Pose(x=_f(r["x"]), y=_f(r["y"]), z=_f(r["z"])), payload={}))
    for r in _rows(d / "camera_index.csv"):
        caps.append(Capture("rov_camera", _f(r["t"]), f"rov_camera:{r['seq']}",
                            Pose(x=_f(r["x"]), y=_f(r["y"]), z=_f(r["z"])),
                            payload={"visibility": _f(r.get("visibility"), 0.0)}))
    return caps


def _still(bag_dir: str, kind: str, seq: str) -> Optional[str]:
    try:
        p = rov_dir(bag_dir) / "frames" / f"{kind}_{int(seq):05d}.png"
    except ValueError:
        return None
    return p.as_posix() if p.exists() else None


def gt_detections(bag_dir: str, mission_id: str, sonar_caps: list[Capture]) -> list[Detection]:
    """ROV detections for the underwater GT defects (z below the water line).

    Each is attached to the sonar ping whose ROV pose was closest to the defect
    (within SONAR_RANGE_M). Defects never in sonar range are not invented.
    """
    from .rosbag import GT_CLASS_MAP, GT_SEVERITY_CONF, gt_platform

    gt_path = Path(bag_dir) / "defects_gt.json"
    if not gt_path.exists() or not sonar_caps:
        return []
    out = []
    for g in json.loads(gt_path.read_text(encoding="utf-8-sig")).get("defects", []):
        pos = g.get("position") or [0, 0, 0]
        if gt_platform(pos) != "rov":
            continue
        cls = GT_CLASS_MAP.get(g.get("type"))
        if cls is None:
            continue
        ping = min(sonar_caps, key=lambda c: math.dist((c.pose.x, c.pose.y, c.pose.z), pos))
        if math.dist((ping.pose.x, ping.pose.y, ping.pose.z), pos) > SONAR_RANGE_M:
            continue
        det = make_detection(
            mission_id=mission_id, platform="rov", cls=cls,
            confidence=GT_SEVERITY_CONF.get(int(g.get("severity", 2)), 0.7),
            timestamp_utc=ping.timestamp_utc, pose=ping.pose,
            source_frame_id=ping.capture_id, evidence_frame_ids=[ping.capture_id],
            model_version="ground-truth@defects_gt",
        )
        det._gt_position = pos                       # type: ignore[attr-defined]
        det._gt_element = g.get("element", "")       # type: ignore[attr-defined]
        det._gt_severity = int(g.get("severity", 0))  # type: ignore[attr-defined]
        det._gt_id = g.get("id", "")                 # type: ignore[attr-defined]
        det.location.structure.elevation_or_depth = round(float(pos[2]), 3)
        out.append(det)
    return out


def confirm_with_camera(index: TimePoseIndex, detections: list[Detection],
                        min_visibility: float = ROV_MIN_VISIBILITY) -> None:
    """Clarity-gated camera confirm: a co-located ROV camera frame (time/pose
    gate) confirms a sonar detection only if its visibility clears the gate."""
    for det in detections:
        if det.platform != "rov":
            continue
        frame = index.retrace("rov_camera", det.timestamp_utc, det.pose)
        vis = (frame.payload or {}).get("visibility", 0.0) if frame else 0.0
        det._camera_visibility = vis if frame else None   # type: ignore[attr-defined]
        if frame is None or vis < min_visibility:
            det.confirmed = False
            continue
        det.confirmed = True
        if frame.capture_id not in det.evidence_frame_ids:
            det.evidence_frame_ids.append(frame.capture_id)


def evidence_stills(bag_dir: str, det: Detection) -> list[str]:
    """Paths of the extracted sonar/camera stills backing a ROV detection."""
    out = []
    for fid in det.evidence_frame_ids:
        kind, _, seq = fid.partition(":")
        p = _still(bag_dir, "sonar" if kind == "sonar" else "camera", seq)
        if p:
            out.append(p)
    return out


# --- health (dashboard tab 2) ---------------------------------------------------

def rov_health(bag_dir: str, detections: Optional[list[Detection]] = None) -> Optional[dict]:
    """Real ROV health + data quality from the extract (None if absent)."""
    d = rov_dir(bag_dir)
    tel = _rows(d / "telemetry.csv")
    status = _rows(d / "status.csv")
    if not tel:
        return None
    t0, t1 = _f(tel[0]["t"]), _f(tel[-1]["t"])
    duration = max(t1 - t0, 0.0)
    drift = [_f(r.get("drift_m")) for r in tel]
    depths = [-_f(r["z"]) for r in tel]

    per_area: dict[str, int] = {}
    locked = 0
    nav_err = []
    for r in status:
        area = r.get("area") or "unknown"
        per_area[area] = per_area.get(area, 0) + 1
        locked += str(r.get("dvl_lock")) == "True"
        nav_err.append(_f(r.get("nav_err_m")))
    n_status = len(status) or 1
    transit = sum(c for a, c in per_area.items() if a in ("transit", "surface", "unknown"))
    step = duration / n_status if status else 0.0

    def pct(c, n):
        return round(100.0 * c / n, 1) if n else 0.0

    sonar = _rows(d / "sonar_index.csv")
    cam = _rows(d / "camera_index.csv")
    vis = sorted(_f(r.get("visibility")) for r in cam)
    clear = sum(1 for v in vis if v >= ROV_MIN_VISIBILITY)
    coop = _rows(d / "coop_tasks.csv")
    rov_dets = [x for x in (detections or []) if x.platform == "rov"]
    confirmed = sum(1 for x in rov_dets if x.confirmed)

    issues = []
    if pct(locked, n_status) < 95:
        issues.append(f"DVL bottom lock held {pct(locked, n_status)}% of status samples")
    if drift and max(drift) > 1.0:
        issues.append(f"DVL/EKF drift vs truth reached {max(drift):.2f} m")
    if cam and pct(clear, len(cam)) < 50:
        issues.append(f"only {pct(clear, len(cam))}% of camera frames clear the visibility gate")
    grade = "high" if not issues else ("moderate" if len(issues) == 1 else "degraded")

    return {
        "platform": "rov",
        "model": "BlueROV2 Heavy (ArduSub SITL, DVL nav, multibeam sonar + camera)",
        "is_filler": False,
        "mission": {"duration_s": round(duration, 1),
                    "telemetry_hz": round(len(tel) / duration, 1) if duration else None,
                    "samples": len(tel)},
        "utilization": {
            "inspecting_pct": pct(n_status - transit, n_status) if status else 0.0,
            "transit_pct": pct(transit, n_status) if status else 0.0,
            "per_element": [{"element": a, "steps": c, "seconds": round(c * step, 1),
                             "share_pct": pct(c, n_status)}
                            for a, c in sorted(per_area.items(), key=lambda kv: -kv[1])],
        },
        "nav": {
            "dvl_lock_pct": pct(locked, n_status),
            "avg_nav_err_m": round(sum(nav_err) / len(nav_err), 3) if nav_err else None,
            "avg_drift_m": round(sum(drift) / len(drift), 3) if drift else None,
            "max_drift_m": round(max(drift), 3) if drift else None,
            "max_depth_m": round(max(depths), 2) if depths else None,
        },
        "sonar": {"frames": len(sonar),
                  "rate_hz": round(len(sonar) / duration, 2) if duration else None,
                  "note": f"{len(sonar)} multibeam pings over {round(duration)} s"},
        "camera": {"frames": len(cam),
                   "avg_visibility": round(sum(vis) / len(vis), 3) if vis else None,
                   "clear_pct": pct(clear, len(cam)),
                   "confirmed_pct": pct(confirmed, len(rov_dets)) if rov_dets else None,
                   "note": f"{clear}/{len(cam)} frames clear the visibility gate "
                           f"(RMS contrast >= {ROV_MIN_VISIBILITY})"},
        "coop": {"tasks": len(coop), "defects": sorted({r["defect_id"] for r in coop})},
        "dives": len({r["defect_id"] for r in coop}),
        "data_quality": {"grade": grade, "issues": issues},
    }


# --- sonar model detections ---------------------------------------------------
#
# /rov/sonar/image is a 560x300 rendering of a 130 deg / 12 m multibeam fan.
# Measured from the frames (out of the bag, not assumed): apex at pixel
# (280, 292), outer arc 291 px from the apex, edges at +/-65 deg.
FAN_APEX_PX = (280.0, 292.0)
FAN_RADIUS_PX = 291.0
FAN_RANGE_M = 12.0
FAN_HALF_DEG = 65.0
UNWARP_SIZE = (291, 512)            # rows = range bins, cols = bearing bins

# The generic model returns many boxes that cover most of the fan -- it is
# reacting to the fan shape, not a target. Boxes larger than this share of the
# image are dropped (measured: whole-fan boxes are 0.6-1.0 of the frame).
SONAR_MAX_BOX_FRAC = 0.25
# Repeated hits on the same spot from consecutive pings are one anomaly.
SONAR_CLUSTER_M = 1.5
# An anomaly must be seen on at least this many pings (single-ping speckle is
# not reported). The fan updates at ~3.5 Hz, so 2 pings is ~0.6 s.
SONAR_MIN_HITS = 2
SONAR_MODES = ("raw", "unwarp")


def unwarp_fan(img):
    """Fan image -> rectangular range x bearing image (far range at the top),
    the layout side-scan models are trained on. Needs numpy + OpenCV."""
    import cv2  # type: ignore
    import numpy as np  # type: ignore
    rows, cols = UNWARP_SIZE
    rr, bb = np.meshgrid(np.arange(rows), np.linspace(-FAN_HALF_DEG, FAN_HALF_DEG, cols), indexing="ij")
    r_px = rr * (FAN_RADIUS_PX / rows)
    ang = np.deg2rad(bb)
    xs = (FAN_APEX_PX[0] + r_px * np.sin(ang)).astype(np.float32)
    ys = (FAN_APEX_PX[1] - r_px * np.cos(ang)).astype(np.float32)
    return cv2.remap(img, xs, ys, cv2.INTER_LINEAR)[::-1].copy()


def box_to_polar(box, mode: str, img_shape) -> tuple[float, float]:
    """Box centre -> (range_m, bearing_deg, starboard positive)."""
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    if mode == "unwarp":
        rows, cols = img_shape[:2]
        rng = (1.0 - cy / max(rows - 1, 1)) * FAN_RANGE_M
        brg = -FAN_HALF_DEG + 2 * FAN_HALF_DEG * cx / max(cols - 1, 1)
        return rng, brg
    dx, dy = cx - FAN_APEX_PX[0], FAN_APEX_PX[1] - cy
    return math.hypot(dx, dy) / FAN_RADIUS_PX * FAN_RANGE_M, math.degrees(math.atan2(dx, dy))


def polar_to_world(rov_xyz, yaw_rad: float, range_m: float, bearing_deg: float) -> list:
    """Place a sonar return in the world frame (X east, Y north). The fan is
    treated as horizontal at the ROV's depth -- the bag gives no sonar tilt, so
    the return's height is not recoverable from the 2-D image."""
    a = yaw_rad - math.radians(bearing_deg)
    return [round(rov_xyz[0] + range_m * math.cos(a), 3),
            round(rov_xyz[1] + range_m * math.sin(a), 3),
            round(rov_xyz[2], 3)]


def _box_reject(box, img_shape, mode) -> str:
    h, w = img_shape[:2]
    frac = (box[2] - box[0]) * (box[3] - box[1]) / float(w * h)
    if frac > SONAR_MAX_BOX_FRAC:
        return f"box covers {frac:.2f} of frame"
    rng, brg = box_to_polar(box, mode, img_shape)
    if rng > FAN_RANGE_M or abs(brg) > FAN_HALF_DEG:
        return "centre outside the fan"
    return ""


CACHE_FIELDS = ["t", "seq", "raw_class", "confidence", "x1", "y1", "x2", "y2",
                "range_m", "bearing_deg", "wx", "wy", "wz", "rejected", "model_version"]


def _cache_path(bag_dir: str, mode: str) -> Path:
    return rov_dir(bag_dir) / f"sonar_detections_{mode}.csv"


def sonar_cache_complete(bag_dir: str, mode: str) -> bool:
    """A cache is only reused if the run that wrote it finished. A killed run
    used to leave a partial CSV that later runs silently reused."""
    return _cache_path(bag_dir, mode).exists() and _cache_path(bag_dir, mode).with_suffix(".done").exists()


def run_sonar_model(bag_dir: str, detector, mode: str = "raw", stride: int = 1) -> Path:
    """Run the sonar model over every `stride`-th /rov/sonar/image ping in the
    .mcap and write every box (kept or rejected, with the reason) to
    rov/sonar_detections_<mode>.csv. Needs mcap, mcap-ros2-support, numpy,
    OpenCV and the model weights."""
    import numpy as np  # type: ignore
    from mcap.reader import make_reader  # type: ignore
    from mcap_ros2.decoder import DecoderFactory  # type: ignore

    from .rosbag import ROV_TOPICS, _find_mcap

    if mode not in SONAR_MODES:
        raise ValueError(f"mode must be one of {SONAR_MODES}")
    index = {int(r["seq"]): r for r in _rows(rov_dir(bag_dir) / "sonar_index.csv")}
    final = _cache_path(bag_dir, mode)
    final.with_suffix(".done").unlink(missing_ok=True)
    out = final.with_suffix(".part")
    try:
        # The entry points pin OMP_NUM_THREADS=1 (Windows torch/numpy guard),
        # which leaves torch on one core: ~1 s per ping. Inference itself is
        # safe to parallelise.
        import os
        import torch  # type: ignore
        torch.set_num_threads(max(1, (os.cpu_count() or 2) - 1))
    except ImportError:
        pass
    import time
    t0 = time.time()
    done = 0
    dec = DecoderFactory()
    decode = None
    seq = -1
    with open(_find_mcap(bag_dir), "rb") as f, out.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(CACHE_FIELDS)
        for schema, ch, msg in make_reader(f).iter_messages(topics=[ROV_TOPICS["sonar"]]):
            seq += 1
            if seq % stride or seq not in index:
                continue
            decode = decode or dec.decoder_for(ch.message_encoding, schema)
            m = decode(msg.data)
            img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, ::-1].copy()
            if mode == "unwarp":
                img = unwarp_fan(img)
            row = index[seq]
            pos = (_f(row["x"]), _f(row["y"]), _f(row["z"]))
            for p in detector.predict_image(img):
                box = p["box"]
                rng, brg = box_to_polar(box, mode, img.shape)
                wx, wy, wz = polar_to_world(pos, _f(row.get("yaw")), rng, brg)
                w.writerow([row["t"], seq, p.get("raw_class", ""), round(p["confidence"], 4),
                            *(round(v, 1) for v in box), round(rng, 3), round(brg, 2),
                            wx, wy, wz, _box_reject(box, img.shape, mode), detector.model_version])
            fh.flush()
            done += 1
            if done % 100 == 0:
                print(f"    sonar model: {done}/{len(index) // stride} pings, "
                      f"{(time.time() - t0) / done:.2f} s/ping", flush=True)
    out.replace(final)
    final.with_suffix(".done").write_text(f"pings={done} stride={stride} mode={mode}\n", encoding="utf-8")
    return final


def sonar_model_detections(bag_dir: str, mission_id: str, mode: str = "raw",
                           min_conf: float = 0.0) -> list[Detection]:
    """Clustered sonar_anomaly detections from the cached model output.

    Drops rejected boxes, then greedily clusters the rest by world position
    (highest confidence first, SONAR_CLUSTER_M) and keeps clusters seen on at
    least SONAR_MIN_HITS pings. Each anomaly cites its strongest pings.
    """
    rows = [r for r in _rows(_cache_path(bag_dir, mode))
            if not r.get("rejected") and _f(r["confidence"]) >= min_conf]
    rows.sort(key=lambda r: (-_f(r["confidence"]), int(r["seq"])))
    clusters: list[dict] = []
    for r in rows:
        p = (_f(r["wx"]), _f(r["wy"]), _f(r["wz"]))
        for c in clusters:
            if math.dist(c["pos"], p) <= SONAR_CLUSTER_M:
                c["rows"].append(r)
                break
        else:
            clusters.append({"pos": p, "rows": [r]})
    index = {int(r["seq"]): r for r in _rows(rov_dir(bag_dir) / "sonar_index.csv")}
    out = []
    for c in clusters:
        pings = sorted({int(r["seq"]) for r in c["rows"]})
        if len(pings) < SONAR_MIN_HITS:
            continue
        best = c["rows"][0]
        ping = index.get(int(best["seq"]), best)
        det = make_detection(
            mission_id=mission_id, platform="rov", cls="sonar_anomaly",
            confidence=_f(best["confidence"]), timestamp_utc=_f(best["t"]),
            pose=Pose(x=_f(ping.get("x")), y=_f(ping.get("y")), z=_f(ping.get("z"))),
            source_frame_id=f"sonar:{best['seq']}",
            evidence_frame_ids=[f"sonar:{s}" for s in sorted(
                {int(r["seq"]) for r in c["rows"]}, key=lambda s: -max(
                    _f(r["confidence"]) for r in c["rows"] if int(r["seq"]) == s))[:10]],
            model_version=best.get("model_version") or "sonar-yolo",
        )
        det.world_pos = list(c["pos"])
        det.location.structure.elevation_or_depth = round(c["pos"][2], 3)
        det._raw_class = best.get("raw_class")     # type: ignore[attr-defined]
        det._sonar_hits = len(pings)               # type: ignore[attr-defined]
        det._range_m = _f(best["range_m"])         # type: ignore[attr-defined]
        det._no_element = True                     # type: ignore[attr-defined]
        out.append(det)
    return out


def sonar_cache_stats(bag_dir: str, mode: str = "raw") -> dict:
    rows = _rows(_cache_path(bag_dir, mode))
    rejected: dict[str, int] = {}
    for r in rows:
        if r.get("rejected"):
            key = "too large" if "covers" in r["rejected"] else r["rejected"]
            rejected[key] = rejected.get(key, 0) + 1
    return {"boxes": len(rows), "pings_with_boxes": len({r["seq"] for r in rows}),
            "kept": sum(1 for r in rows if not r.get("rejected")), "rejected": rejected}
