#!/usr/bin/env python3
"""Build a git-sized sample of a recorded mission under sample_data/.

    python make_sample.py --bag ../navfix/bridge-inspection/data/run_20261024_134808

The full bags are 1.5-2.2 GB (GitHub rejects files over 100 MB). The pipeline
does not need the .mcap once the frames are extracted, so the sample keeps:

  inspection_frames/  drone stills + sidecars + telemetry.csv  (as recorded)
  rov/                ROV extract from extract_rov.py           (if present)
  defects_gt.json, README.md, metadata.yaml, coop_log.txt
  map/<name>.npy      each LiDAR/sonar map, FULL resolution within
                      --keep-radius of every GT defect and every inspection-frame
                      aim point (where geometry is measured), and voxel-thinned to
                      --voxel elsewhere (only the 3D view uses those points)

Geometry measured from the sample therefore matches the full bag at every
defect and frame position. Needs numpy.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from pipeline.rosbag import _surface_point  # noqa: E402

MAP_NAMES = ("lidar_map_world", "drone_lidar_world", "rov_sonar_world")
TOP_FILES = ("defects_gt.json", "README.md", "metadata.yaml", "coop_log.txt")
MAX_FILE_MB = 50


def anchor_points(bag: Path) -> list:
    pts = []
    gt = bag / "defects_gt.json"
    if gt.exists():
        pts += [d["position"] for d in json.loads(gt.read_text(encoding="utf-8-sig")).get("defects", [])
                if d.get("position")]
    for sc in sorted((bag / "inspection_frames").glob("[0-9][0-9][0-9][0-9].json")):
        m = json.loads(sc.read_text(encoding="utf-8-sig"))
        for p in (m.get("target"), _surface_point(m.get("drone"), m.get("target"), m.get("standoff_m"))):
            if p:
                pts.append(p)
    return pts


def thin_map(src: Path, dst: Path, anchors, keep_radius: float, voxel: float) -> str:
    import numpy as np
    pts = np.load(str(src))[:, :3].astype(np.float32)
    keep = np.zeros(len(pts), dtype=bool)
    for a in anchors:
        a = np.asarray(a, dtype=np.float32)
        keep |= (np.abs(pts - a) <= keep_radius).all(axis=1)
    rest = pts[~keep]
    keys = np.floor(rest / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    out = np.concatenate([pts[keep], rest[np.sort(idx)]])
    np.save(str(dst), out)
    return f"{src.name}: {len(pts):,} -> {len(out):,} pts ({keep.sum():,} kept at full res)"


def main() -> int:
    ap = argparse.ArgumentParser(description="Build a git-sized mission sample")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default=None, help="default sample_data/<bag name>")
    ap.add_argument("--keep-radius", type=float, default=1.5,
                    help="half-size (m) of the full-resolution box around each anchor")
    ap.add_argument("--voxel", type=float, default=0.25, help="voxel size (m) elsewhere")
    args = ap.parse_args()

    bag = Path(args.bag)
    out = Path(args.out) if args.out else HERE / "sample_data" / bag.name
    out.mkdir(parents=True, exist_ok=True)

    for name in TOP_FILES:
        if (bag / name).exists():
            shutil.copy2(bag / name, out / name)
    frames = bag / "inspection_frames"
    (out / "inspection_frames").mkdir(exist_ok=True)
    for f in frames.iterdir():
        if f.is_file() and (f.name == "telemetry.csv" or f.stem.isdigit()):
            shutil.copy2(f, out / "inspection_frames" / f.name)
    if (bag / "rov").is_dir():
        shutil.copytree(bag / "rov", out / "rov", dirs_exist_ok=True)

    anchors = anchor_points(bag)
    (out / "map").mkdir(exist_ok=True)
    for name in MAP_NAMES:
        src = bag / "map" / f"{name}.npy"
        if src.exists():
            print("  " + thin_map(src, out / "map" / src.name, anchors, args.keep_radius, args.voxel))

    files = [p for p in out.rglob("*") if p.is_file()]
    total = sum(p.stat().st_size for p in files) / 2**20
    big = [p for p in files if p.stat().st_size > MAX_FILE_MB * 2**20]
    print(f"wrote {out}: {len(files)} files, {total:.1f} MB")
    if big:
        print(f"ERROR: files over {MAX_FILE_MB} MB: {[p.name for p in big]}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
