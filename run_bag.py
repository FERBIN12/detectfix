#!/usr/bin/env python3
"""Run the pipeline end-to-end on a real ROS2 rosbag2 (.mcap) mission.

    python run_bag.py --bag ../data/run_20261024_123507

Decodes the real bag (/odom poses, /lidar/points for retrace, /camera/rgb
frames), uses the mission's ground-truth defects as the detection source
(stand-in for the trained camera model), then runs retrace -> LiDAR depth ->
correlate -> severity -> report -> dashboard, exactly like the main flow.
"""
import argparse
import json
import os
import sys
from pathlib import Path

# Guard against the torch<->numpy OpenMP/MKL runtime clash on Windows, which can
# hard-crash the process when numpy does heavy linear algebra after torch is
# imported. Must be set before numpy/torch load.
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pipeline import agents, correlation, detection, indexing, lidar, severity, weights  # noqa: E402
from pipeline import rosbag as rb  # noqa: E402
from pipeline.orchestrator import compute_metrics  # noqa: E402


def _defect_position(d):
    """World position for a detection: GT position if present (gt detector),
    else the drone->target surface projection (model detector), else target,
    else drone pose."""
    for attr in ("_gt_position", "_surface_pos", "_frame_target"):
        pos = getattr(d, attr, None)
        if pos is not None:
            return pos
    return [d.pose.x, d.pose.y, d.pose.z]


def element_map_from_detections(detections) -> dict:
    """Build an element map from each detection's world position + element label
    so CORRELATE assigns detections to bridge elements. Works for both the real
    model path (element from the frame sidecar) and the GT path."""
    elements = []
    for d in detections:
        pos = _defect_position(d)
        el = getattr(d, "_gt_element", "") or getattr(d, "_frame_element", "")
        elements.append({"element_id": el, "face": "", "x": pos[0], "y": pos[1],
                         "z": pos[2], "station": round(pos[0], 2), "offset": round(pos[1], 2)})
    return {"elements": elements}


def main() -> int:
    ap = argparse.ArgumentParser(description="Run the pipeline on a real ROS2 bag")
    ap.add_argument("--bag", required=True, help="path to the rosbag2 directory or .mcap")
    ap.add_argument("--mission-id", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-dt-s", type=float, default=2.0)
    ap.add_argument("--max-pose-m", type=float, default=5.0)
    ap.add_argument("--detector", choices=["model", "gt"], default="model",
                    help="'model' runs the real YOLOv8-seg over inspection frames; "
                         "'gt' uses defects_gt.json as the detection source")
    ap.add_argument("--camera-weights", default=None,
                    help="path to the YOLOv8-seg defect weights (auto-downloaded if omitted)")
    ap.add_argument("--conf", type=float, default=0.25)
    args = ap.parse_args()

    bag = args.bag
    mission_id = args.mission_id or Path(bag).name

    print(f"decoding bag {bag} ...", flush=True)
    captures, pose_tl = rb.decode_bag(bag)
    rgb = sum(1 for c in captures if c.kind == "rgb")
    lid = sum(1 for c in captures if c.kind == "lidar")
    span = pose_tl.span()
    print(f"  decoded: {rgb} rgb frames, {lid} lidar clouds, {len(pose_tl)} odom poses")
    if span[0] is not None:
        print(f"  pose span: {span[1] - span[0]:.1f} s")

    index = indexing.build_index(captures, max_dt_s=args.max_dt_s, max_pose_m=args.max_pose_m)

    # DETECT
    if args.detector == "model":
        wpath = weights.resolve_camera_weights(args.camera_weights)
        print(f"  camera weights: {wpath}")
        camera = detection.CameraDetector(weights=wpath, conf_threshold=args.conf)
        detections, dstats = rb.detect_inspection_frames(bag, mission_id, camera,
                                                         conf_threshold=args.conf)
        print(f"  real detections over {dstats['frames_total']} frames: "
              f"{dstats['detections']} defects in {dstats['frames_with_defects']} frames")
    else:
        detections = rb.gt_detections(bag, mission_id, captures, pose_tl)
        print(f"  detections (from ground truth): {len(detections)}")

    # LiDAR DEPTH ESTIMATION off the real accumulated world map, cropped around
    # each defect's world position (the honest geometry source for this bag).
    map_dir = Path(bag) / "map"
    world = lidar.WorldCloud.load(str(map_dir)) if map_dir.exists() else None
    positions = {d.detection_id: _defect_position(d) for d in detections}
    positions = {k: v for k, v in positions.items() if v is not None}
    for d in detections:                      # persist world pos for the 3D viewer
        d.world_pos = positions.get(d.detection_id)
    if world is not None:
        lidar.measure_from_world_map(world, detections, positions, half=0.3)
        src = "world map"
    else:
        lidar.run_lidar_depth(index, detections)
        src = "per-frame cloud"
    measured = sum(1 for d in detections if d.geometry.is_measured())
    print(f"  lidar depth measured on: {measured}/{len(detections)} (source: {src})")

    # CORRELATE + SEVERITY
    emap = element_map_from_detections(detections)
    detections = correlation.run_correlation(detections, emap, prior=[])
    severity.run_severity(detections)

    metrics = compute_metrics(detections)
    report = agents.draft_report(detections)
    dashboard = agents.build_dashboard(report, metrics)

    result = {
        "mission_id": mission_id,
        "detections": [d.to_dict() for d in detections],
        "metrics": metrics,
        "report": report,
        "dashboard": dashboard,
        "index_counts": {k: index.count(k) for k in indexing.STREAM_KINDS},
    }

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"wrote {args.out}")

    m = metrics
    print(f"\nmission {mission_id}: {m['defect_count']} defects "
          f"({m['severe']} severe, {m['poor']} poor, {m['fair']} fair) "
          f"| backend={report['backend']} report={report['source']}")
    print("--- report ---")
    print(report["narrative"])
    print("--- dashboard ---")
    print(dashboard["headline"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
