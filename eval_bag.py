#!/usr/bin/env python3
"""Score a pipeline result against the mission's ground truth.

    python eval_bag.py --result out/sample_model.json --bag sample_data/run_20261024_123507

Prints precision / recall / F1 (class-agnostic and class-aware), per-class
recall and a confidence sweep; --out writes the full, traceable evaluation.
Standard library only. Spec: .kiro/specs/detection-evaluation/.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pipeline import evaluation as ev  # noqa: E402


def _row(name, m):
    return (f"  {name:<15} TP {m['tp']:>3}  FP {m['fp']:>3}  FN {m['fn']:>3}  "
            f"P {m['precision']:.2f}  R {m['recall']:.2f}  F1 {m['f1']:.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Evaluate detections against defects_gt.json")
    ap.add_argument("--result", required=True, help="pipeline result JSON (run_bag.py --out)")
    ap.add_argument("--bag", default=None, help="bag/sample dir containing defects_gt.json")
    ap.add_argument("--gt", default=None, help="explicit path to defects_gt.json")
    ap.add_argument("--radius", type=float, default=ev.DEFAULT_RADIUS_M,
                    help=f"match radius in metres (default {ev.DEFAULT_RADIUS_M})")
    ap.add_argument("--no-element-gate", action="store_true",
                    help="allow matches across bridge elements")
    ap.add_argument("--thresholds", default=",".join(str(t) for t in ev.DEFAULT_THRESHOLDS),
                    help="comma-separated confidence thresholds for the sweep")
    ap.add_argument("--out", default=None, help="write the full evaluation JSON here")
    args = ap.parse_args()

    gt_path = Path(args.gt) if args.gt else (Path(args.bag) / "defects_gt.json" if args.bag else None)
    if gt_path is None:
        print("error: give --bag or --gt", file=sys.stderr)
        return 2
    try:
        gts, skipped = ev.load_ground_truth(gt_path)
    except (FileNotFoundError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    try:
        result = json.loads(Path(args.result).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"error: cannot read result {args.result}: {exc}", file=sys.stderr)
        return 2
    try:
        thresholds = [float(t) for t in args.thresholds.split(",") if t.strip()]
    except ValueError:
        print(f"error: bad --thresholds {args.thresholds!r}", file=sys.stderr)
        return 2

    preds = ev.predictions_from_result(result)
    report = ev.evaluate(preds, gts, radius_m=args.radius,
                         element_gate=not args.no_element_gate, thresholds=thresholds)
    report = {"mission_id": result.get("mission_id", ""),
              "result": str(Path(args.result).as_posix()),
              "ground_truth": {"path": gt_path.as_posix(), "count": len(gts), "skipped": skipped},
              **report}

    print(f"evaluation: {report['mission_id']}  ({len(preds)} predictions vs {len(gts)} GT, "
          f"radius {args.radius} m, element gate {'on' if not args.no_element_gate else 'off'})")
    _, gt_sourced = ev.split_by_source(preds)
    if gt_sourced:
        plats = sorted({p.platform for p in gt_sourced})
        print(f"  NOTE: {len(gt_sourced)} predictions are ground-truth-sourced ({', '.join(plats)}); "
              "their scores are a self-check, not detector quality")
    report["gt_sourced_predictions"] = len(gt_sourced)
    print(_row("class-agnostic", report["class_agnostic"]))
    print(_row("class-aware", report["class_aware"]))
    for plat, r in report["per_platform"].items():
        print(_row(f"{plat} ({r['gt']} GT)", r["class_agnostic"]))
    print("  per-class recall (class-aware):")
    for cls, r in report["per_class_recall"].items():
        print(f"    {cls:<16} {r['found']}/{r['gt']}  ({r['recall']:.2f})")
    print("  confidence sweep (class-agnostic):")
    for r in report["sweep"]:
        print(f"    conf >= {r['threshold']:<5} TP {r['tp']:>3}  FP {r['fp']:>3}  "
              f"P {r['precision']:.2f}  R {r['recall']:.2f}")
    ca = report["class_agnostic"]
    print(f"  matched: {', '.join(p['gt_id'] + '@' + str(p['distance_m']) + 'm' for p in ca['pairs']) or 'none'}")
    print(f"  missed GT: {', '.join(ca['missed']) or 'none'}")

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
