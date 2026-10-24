#!/usr/bin/env python3
"""Export a web-loadable data bundle for the bridge-inspection dashboard.

Produces, under web/data/:
  cloud.bin       voxel-downsampled LiDAR world map as Float32 [x,y,z,...]
  cloud.json      {count, bounds, center} metadata for the viewer
  result.json     the pipeline result (detections, metrics, report, dashboard)
  frames.json     manifest of inspection frames (png name, element, pose, dets)

The point cloud is downsampled so the browser stays smooth; defect positions
are kept at full precision. Run after a pipeline run that wrote the result JSON.
"""
import argparse
import json
import os
import struct
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def voxel_downsample(points, voxel=0.1, max_points=120000):
    """Keep one point per voxel cell; cap the total. Pure numpy."""
    import numpy as np
    p = np.asarray(points, dtype=np.float64)
    keys = np.floor(p[:, :3] / voxel).astype(np.int64)
    # unique voxel cell -> first point index
    _, idx = np.unique(keys, axis=0, return_index=True)
    out = p[np.sort(idx)][:, :3]
    if len(out) > max_points:
        step = len(out) // max_points + 1
        out = out[::step]
    return out.astype(np.float32)


def export_cloud(map_dir: Path, web_data: Path, voxel: float):
    """Drone LiDAR map plus, for drone+ROV missions, the ROV sonar map (below
    the water line), merged into one cloud for the 3D view."""
    import numpy as np
    parts = []
    for name in ("lidar_map_world", "drone_lidar_world", "rov_sonar_world"):
        npy = map_dir / f"{name}.npy"
        if npy.exists() and not (name == "drone_lidar_world" and parts):
            parts.append(np.load(str(npy))[:, :3])
            print(f"  cloud part: {npy.name} ({len(parts[-1])} points)")
    if not parts:
        print(f"  no cloud in {map_dir}; skipping 3D export")
        return
    pts = np.concatenate(parts)
    ds = voxel_downsample(pts, voxel=voxel)
    (web_data / "cloud.bin").write_bytes(ds.tobytes())
    mn = ds.min(axis=0).tolist()
    mx = ds.max(axis=0).tolist()
    center = [(mn[i] + mx[i]) / 2 for i in range(3)]
    (web_data / "cloud.json").write_text(json.dumps({
        "count": int(len(ds)),
        "bounds": {"min": mn, "max": mx},
        "center": center,
    }), encoding="utf-8")
    print(f"  cloud: {len(pts)} -> {len(ds)} points (voxel {voxel} m)")


def export_frames(bag_dir: Path, web_data: Path, result: dict):
    """Manifest of inspection frames + the detections mapped to each frame."""
    frames_dir = bag_dir / "inspection_frames"
    # group detections by their source frame id (e.g. "frame:0017")
    by_frame = {}
    for d in result.get("detections", []):
        fid = d.get("source_frame_id", "")
        stem = fid.split(":")[-1] if ":" in fid else fid
        by_frame.setdefault(stem, []).append({
            "class": d.get("class"), "confidence": d.get("confidence"),
            "severity": d.get("severity"), "element": d["location"]["structure"]["element_id"],
        })
    frames = []
    for png in sorted(frames_dir.glob("[0-9][0-9][0-9][0-9].png")):
        meta_path = png.with_suffix(".json")
        meta = json.loads(meta_path.read_text(encoding="utf-8-sig")) if meta_path.exists() else {}
        frames.append({
            "name": png.name,
            "element": meta.get("element", ""),
            "drone": meta.get("drone"),
            "standoff_m": meta.get("standoff_m"),
            "gnss": (meta.get("gnss") or {}).get("fix") if meta.get("gnss") else None,
            "detections": by_frame.get(png.stem, []),
        })
    (web_data / "frames.json").write_text(json.dumps({"frames": frames}, indent=2), encoding="utf-8")
    print(f"  frames: {len(frames)} ({sum(1 for f in frames if f['detections'])} with defects)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Export web dashboard data")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--result", default="out/bag_model.json")
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--eval", default=None,
                    help="eval_bag.py --out JSON to show in the accuracy panel (omitted: panel hidden)")
    args = ap.parse_args()

    bag_dir = Path(args.bag)
    web_data = HERE / "web" / "data"
    web_data.mkdir(parents=True, exist_ok=True)

    result = json.loads(Path(args.result).read_text(encoding="utf-8"))
    (web_data / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    eval_out = web_data / "eval.json"
    if args.eval:
        eval_out.write_text(Path(args.eval).read_text(encoding="utf-8"), encoding="utf-8")
    elif eval_out.exists():
        eval_out.unlink()   # never show another mission's accuracy numbers
    print(f"exporting web bundle for {bag_dir.name} ...")
    export_cloud(bag_dir / "map", web_data, args.voxel)
    export_frames(bag_dir, web_data, result)
    print(f"done -> {web_data}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
