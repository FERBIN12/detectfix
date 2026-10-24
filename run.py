#!/usr/bin/env python3
"""Run the bridge-inspection detection pipeline on a recorded mission.

    python run.py                          # mock mode, demo fixture from config.json
    python run.py --mcap bags/mission1.mcap
    python run.py --mode live --camera-weights weights/crack_yolov8.pt \
                  --sonar-weights weights/kd_yolox_vit.pth
    python run.py --out out/mission1.json  # write the full result to a file

Mode:
  mock  (default)  decode a JSON fixture, run mock detectors -> no deps needed
  live             decode a real .mcap and load real model weights

The flow is one-way: it reads a recorded bag and never signals a vehicle.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pipeline import config as cfgmod          # noqa: E402
from pipeline.orchestrator import run_pipeline  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Bridge-inspection detection pipeline")
    ap.add_argument("--config", default=None)
    ap.add_argument("--mode", choices=["mock", "live"], default=None)
    ap.add_argument("--mission-id", default=None)
    ap.add_argument("--mcap", default=None, help="path to the rosbag .mcap (or JSON fixture)")
    ap.add_argument("--camera-weights", default=None, help="YOLOv8-seg crack-detection weights (live)")
    ap.add_argument("--sonar-weights", default=None, help="KD-YOLOX-ViT weights (live)")
    ap.add_argument("--prior-dir", default=None, help="prior inspection cycle dir (detections.json)")
    ap.add_argument("--out", default=None, help="write the full result JSON here")
    args = ap.parse_args()

    cfg = cfgmod.load_config(args.config, args.mode)
    live = cfg.get("live", {})
    mode = cfg.get("mode", "mock")

    mcap = args.mcap or live.get("mcap_path")
    if not mcap:
        print("error: no --mcap given and no live.mcap_path in config", file=sys.stderr)
        return 2

    mission_id = args.mission_id or cfg.get("mission_id", "M-001")
    camera_weights = args.camera_weights or (live.get("camera_weights") if mode == "live" else None)
    sonar_weights = args.sonar_weights or (live.get("sonar_weights") if mode == "live" else None)

    result = run_pipeline(
        mission_id=mission_id,
        mcap_path=mcap,
        camera_weights=camera_weights,
        sonar_weights=sonar_weights,
        prior_dir=args.prior_dir or live.get("prior_dir"),
        data_dir=live.get("data_dir"),
        max_dt_s=cfg.get("retrace", {}).get("max_dt_s", 0.5),
        max_pose_m=cfg.get("retrace", {}).get("max_pose_m", 2.0),
    )

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"wrote {args.out}")

    m = result["metrics"]
    print(f"mission {result['mission_id']}: {m['defect_count']} defects "
          f"({m['severe']} severe, {m['poor']} poor, {m['worsening']} worsening) "
          f"| backend={result['report']['backend']} report={result['report']['source']}")
    print("--- report ---")
    print(result["report"]["narrative"])
    print("--- dashboard ---")
    print(result["dashboard"]["headline"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
