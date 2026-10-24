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

from pipeline import agents, correlation, detection, health, indexing, lidar, severity, weights  # noqa: E402
from pipeline import rov as rov_mod  # noqa: E402
from pipeline import rosbag as rb  # noqa: E402
from pipeline.orchestrator import compute_metrics  # noqa: E402


def _defect_position(d):
    """World position for a detection: GT position if present (gt detector),
    else the drone->target surface projection (model detector), else target,
    else drone pose."""
    if getattr(d, "world_pos", None):          # e.g. sonar anomalies placed from the fan
        return d.world_pos
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
    ap.add_argument("--rov-detector", choices=["auto", "model", "gt", "none"], default="auto",
                    help="ROV detections: 'model' = generic sonar detector over /rov/sonar/image "
                         "(reported as sonar_anomaly); 'gt' = underwater defects_gt.json entries; "
                         "'none' = drone only; 'auto' = follow --detector")
    ap.add_argument("--sonar-weights", default=None,
                    help="sonar detector weights (default: Sonnet Model C, downloaded on first use)")
    ap.add_argument("--sonar-mode", choices=list(rov_mod.SONAR_MODES), default="unwarp",
                    help="feed the model the raw fan or the range x bearing unwarp")
    ap.add_argument("--sonar-conf", type=float, default=0.1)
    ap.add_argument("--sonar-stride", type=int, default=1, help="run the sonar model on every Nth ping")
    ap.add_argument("--rerun-sonar", action="store_true",
                    help="re-run the sonar model even if rov/sonar_detections_<mode>.csv exists")
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

    # ROV streams (drone+ROV missions): sonar pings + ROV camera frames posed by
    # the ROV's own DVL/EKF estimate, from the rov/ extract (extract_rov.py).
    rov_caps = rov_mod.load_captures(bag) if rov_mod.has_rov(bag) else []
    if rov_caps:
        captures = captures + rov_caps
        print(f"  rov: {sum(1 for c in rov_caps if c.kind == 'sonar')} sonar pings, "
              f"{sum(1 for c in rov_caps if c.kind == 'rov_camera')} camera frames")
    elif (Path(bag) / "map" / "rov_sonar_world.npy").exists():
        print("  rov: this bag has ROV data but no rov/ extract; run extract_rov.py first")

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

    rov_source = args.rov_detector if args.rov_detector != "auto" else (
        "gt" if args.detector == "gt" else "model")
    if rov_caps and rov_source == "model":
        cache = rov_mod.rov_dir(bag) / f"sonar_detections_{args.sonar_mode}.csv"
        has_mcap = any(Path(bag).glob("*.mcap"))
        if cache.exists() and not rov_mod.sonar_cache_complete(bag, args.sonar_mode):
            print(f"  rov: {cache.name} is from an unfinished run; ignoring it")
            if not has_mcap:
                cache = cache.with_suffix(".incomplete")   # forces the "cannot run" branch
        if has_mcap and (args.rerun_sonar or not rov_mod.sonar_cache_complete(bag, args.sonar_mode)):
            swpath = weights.resolve_sonar_weights(args.sonar_weights)
            print(f"  sonar weights: {swpath} (mode {args.sonar_mode}, conf {args.sonar_conf}) ...",
                  flush=True)
            sonar = detection.SonarDetector(weights=swpath, conf_threshold=args.sonar_conf)
            rov_mod.run_sonar_model(bag, sonar, mode=args.sonar_mode, stride=args.sonar_stride)
        elif not cache.exists():
            print(f"  rov: no .mcap and no {cache.name}; cannot run the sonar model here")
        if cache.exists():
            st = rov_mod.sonar_cache_stats(bag, args.sonar_mode)
            rov_dets = rov_mod.sonar_model_detections(bag, mission_id, mode=args.sonar_mode,
                                                      min_conf=args.sonar_conf)
            rov_mod.confirm_with_camera(index, rov_dets)
            detections = detections + rov_dets
            print(f"  sonar model: {st['boxes']} boxes on {st['pings_with_boxes']} pings, "
                  f"{st['kept']} kept (rejected: {st['rejected']}) -> {len(rov_dets)} anomalies "
                  f"(>= {rov_mod.SONAR_MIN_HITS} pings, {rov_mod.SONAR_CLUSTER_M} m clusters), "
                  f"{sum(1 for d in rov_dets if d.confirmed)} camera-confirmed")
    elif rov_caps and rov_source == "gt":
        sonar_caps = [c for c in rov_caps if c.kind == "sonar"]
        rov_dets = rov_mod.gt_detections(bag, mission_id, sonar_caps)
        rov_mod.confirm_with_camera(index, rov_dets)
        detections = detections + rov_dets
        print(f"  rov detections (from ground truth, no sonar model): {len(rov_dets)}, "
              f"{sum(1 for d in rov_dets if d.confirmed)} camera-confirmed "
              f"(visibility >= {rov_mod.ROV_MIN_VISIBILITY})")
    elif rov_caps:
        print("  rov detections: none (--rov-detector none)")

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
    measured = sum(1 for d in detections if d.geometry.is_measured() and d.platform == "drone")
    n_drone = sum(1 for d in detections if d.platform == "drone")
    print(f"  lidar depth measured on: {measured}/{n_drone} drone detections (source: {src})")
    if any(d.platform == "rov" for d in detections):
        sonar_map = lidar.WorldCloud.load_sonar(str(map_dir)) if map_dir.exists() else None
        lidar.measure_from_world_map(sonar_map, detections, positions, half=0.3, platform="rov")
        n_rov = sum(1 for d in detections if d.platform == "rov")
        m_rov = sum(1 for d in detections if d.platform == "rov" and d.geometry.is_measured())
        print(f"  sonar surface roughness measured on: {m_rov}/{n_rov} rov detections")

    # CORRELATE + SEVERITY
    emap = element_map_from_detections(detections)
    detections = correlation.run_correlation(detections, emap, prior=[])
    severity.run_severity(detections)

    metrics = compute_metrics(detections)
    report = agents.draft_report(detections)
    dashboard = agents.build_dashboard(report, metrics)

    # Platform health + data quality (Tab 2). Drone is real from telemetry; the
    # ROV is real when a rov/ extract exists, explicit filler otherwise.
    platform_health = health.build_health(bag, detections)
    dh = platform_health.get("drone") or {}
    if dh:
        print(f"  drone health: {dh['mission']['duration_s']}s mission, "
              f"{dh['utilization']['inspecting_pct']}% inspecting, "
              f"GNSS {dh['gnss']['quality']}, data {dh['data_quality']['grade']}")
    rh = platform_health.get("rov") or {}
    if rh and not rh.get("is_filler"):
        print(f"  rov health: {rh['mission']['duration_s']}s, DVL lock {rh['nav']['dvl_lock_pct']}%, "
              f"drift avg/max {rh['nav']['avg_drift_m']}/{rh['nav']['max_drift_m']} m, "
              f"camera clear {rh['camera']['clear_pct']}%, {rh['dives']} coop dives, "
              f"data {rh['data_quality']['grade']}")

    result = {
        "mission_id": mission_id,
        "detections": [d.to_dict() for d in detections],
        "metrics": metrics,
        "report": report,
        "dashboard": dashboard,
        "health": platform_health,
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
