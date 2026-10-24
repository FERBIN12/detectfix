"""EVALUATION: score pipeline detections against the mission's ground truth.

Spec: .kiro/specs/detection-evaluation/. Standard library only, and no model
takes part -- code enforces, models explain. Every number is traceable to the
listed pairs, false positives and misses.

Matching is greedy closest-pair-first under three gates (radius, element,
optionally class), with a total-order tie-break so the result never depends on
input order.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable, Optional

from .rosbag import GT_CLASS_MAP, gt_platform

# Prediction positions are frame-level: rosbag._surface_point projects from the
# drone along the camera aim ray by the standoff distance, not from the pixel
# mask. On run_20261024_123507 the nearest-GT distance of the 10 model
# detections was 3.8-7.8 m, so a tighter radius would score the projection,
# not the detector. Tighten this once pixel-to-surface ray casting exists.
DEFAULT_RADIUS_M = 8.0

# Classes the camera model and the GT label differently for the same defect.
# The model's "corrosion_stain" is the visual signature GT calls "rust_streak".
CLASS_FAMILIES: tuple[frozenset, ...] = (
    frozenset({"corrosion_stain", "rust_streak"}),
)

DEFAULT_THRESHOLDS = (0.15, 0.2, 0.25, 0.3, 0.4, 0.5)


@dataclass(frozen=True)
class GTDefect:
    id: str
    cls: str
    severity: int
    element: str
    position: tuple
    platform: str = "drone"      # "rov" when below the water line


@dataclass(frozen=True)
class Prediction:
    id: str
    cls: str
    confidence: float
    element: str = ""
    position: Optional[tuple] = None
    platform: str = "drone"
    source: str = "model"        # "ground-truth" when the detection was GT-sourced


@dataclass(frozen=True)
class Pair:
    pred_id: str
    gt_id: str
    distance_m: float


@dataclass
class Matching:
    pairs: list = field(default_factory=list)
    false_positives: list = field(default_factory=list)
    missed: list = field(default_factory=list)
    unlocatable: list = field(default_factory=list)


# --- loading ------------------------------------------------------------------

def load_ground_truth(path) -> tuple[list[GTDefect], list[str]]:
    """Read defects_gt.json into GTDefects in the pipeline's class vocabulary.

    Returns (defects, skipped_ids). Raises FileNotFoundError / ValueError with
    the path in the message when the file is missing or not valid JSON.
    """
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"ground truth not found: {p}")
    try:
        data = json.loads(p.read_text(encoding="utf-8-sig"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"ground truth is not valid JSON: {p} ({exc})") from None
    defects, skipped = [], []
    for i, d in enumerate(data.get("defects", [])):
        gid = str(d.get("id") or f"GT{i:02d}")
        cls = GT_CLASS_MAP.get(d.get("type"))
        pos = d.get("position")
        if cls is None or not pos or len(pos) < 3:
            skipped.append(gid)
            continue
        defects.append(GTDefect(id=gid, cls=cls, severity=int(d.get("severity", 0)),
                                element=str(d.get("element", "")),
                                position=tuple(float(v) for v in pos[:3]),
                                platform=gt_platform(pos)))
    return defects, skipped


def predictions_from_result(result: dict) -> list[Prediction]:
    """Turn a pipeline result JSON's detections[] into Predictions."""
    preds = []
    for d in result.get("detections", []):
        pos = d.get("world_pos")
        element = (((d.get("location") or {}).get("structure") or {}).get("element_id")) or ""
        source = "ground-truth" if str(d.get("model_version", "")).startswith("ground-truth") else "model"
        preds.append(Prediction(
            id=str(d["detection_id"]), cls=str(d.get("class", "")),
            confidence=float(d.get("confidence", 0.0)), element=element,
            position=tuple(float(v) for v in pos[:3]) if pos and len(pos) >= 3 else None,
            platform=str(d.get("platform", "drone")), source=source))
    return preds


# --- matching -----------------------------------------------------------------

def classes_compatible(a: str, b: str) -> bool:
    if a == b:
        return True
    return any(a in fam and b in fam for fam in CLASS_FAMILIES)


def match(preds: Iterable[Prediction], gts: Iterable[GTDefect],
          radius_m: float = DEFAULT_RADIUS_M, class_aware: bool = False,
          element_gate: bool = True) -> Matching:
    """One-to-one greedy matching, closest pair first (Req 2)."""
    preds, gts = list(preds), list(gts)
    m = Matching()
    candidates = []
    for p in preds:
        if p.position is None:
            m.unlocatable.append(p.id)
            continue
        for g in gts:
            if element_gate and p.element and g.element and p.element != g.element:
                continue
            if class_aware and not classes_compatible(p.cls, g.cls):
                continue
            dist = math.dist(p.position, g.position)
            if dist <= radius_m:
                candidates.append((dist, p.id, g.id))
    candidates.sort()
    used_p, used_g = set(), set()
    for dist, pid, gid in candidates:
        if pid in used_p or gid in used_g:
            continue
        used_p.add(pid)
        used_g.add(gid)
        m.pairs.append(Pair(pred_id=pid, gt_id=gid, distance_m=round(dist, 3)))
    m.false_positives = sorted(p.id for p in preds if p.id not in used_p)
    m.missed = sorted(g.id for g in gts if g.id not in used_g)
    m.unlocatable.sort()
    return m


# --- metrics ------------------------------------------------------------------

def _ratio(num: int, den: int) -> float:
    return num / den if den else 0.0


def score(m: Matching, n_preds: int, n_gt: int) -> dict:
    tp = len(m.pairs)
    fp = n_preds - tp
    fn = n_gt - tp
    precision = _ratio(tp, tp + fp)
    recall = _ratio(tp, tp + fn)
    f1 = _ratio(2 * precision * recall, precision + recall) if (precision + recall) else 0.0
    return {"tp": tp, "fp": fp, "fn": fn, "precision": round(precision, 4),
            "recall": round(recall, 4), "f1": round(f1, 4)}


def _mode(preds, gts, **gates) -> dict:
    m = match(preds, gts, **gates)
    out = score(m, len(preds), len(gts))
    out.update({"pairs": [asdict(p) for p in m.pairs], "false_positives": m.false_positives,
                "missed": m.missed, "unlocatable": m.unlocatable})
    return out


def per_class_recall(preds, gts, **gates) -> dict:
    """Class-aware recall per GT class."""
    m = match(preds, gts, class_aware=True, **gates)
    found = {pair.gt_id for pair in m.pairs}
    out: dict = {}
    for g in gts:
        row = out.setdefault(g.cls, {"gt": 0, "found": 0})
        row["gt"] += 1
        row["found"] += g.id in found
    for row in out.values():
        row["recall"] = round(_ratio(row["found"], row["gt"]), 4)
    return dict(sorted(out.items()))


def sweep(preds, gts, thresholds=DEFAULT_THRESHOLDS, class_aware=False, **gates) -> list[dict]:
    """Metrics at each confidence threshold (keep predictions with conf >= t)."""
    rows = []
    for t in sorted(thresholds):
        kept = [p for p in preds if p.confidence >= t]
        row = score(match(kept, gts, class_aware=class_aware, **gates), len(kept), len(gts))
        rows.append({"threshold": t, **row})
    return rows


def evaluate(preds, gts, radius_m: float = DEFAULT_RADIUS_M, element_gate: bool = True,
             thresholds=DEFAULT_THRESHOLDS) -> dict:
    gates = {"radius_m": radius_m, "element_gate": element_gate}
    out = {
        "radius_m": radius_m,
        "element_gate": element_gate,
        "predictions": len(preds),
        "class_agnostic": _mode(preds, gts, class_aware=False, **gates),
        "class_aware": _mode(preds, gts, class_aware=True, **gates),
        "per_class_recall": per_class_recall(preds, gts, **gates),
        "sweep": sweep(preds, gts, thresholds, class_aware=False, **gates),
    }
    # Per platform: the drone camera cannot see underwater defects, so scoring
    # it against them would understate it. Each platform's predictions are
    # scored against the GT defects that platform can observe.
    per_platform = {}
    for plat in sorted({g.platform for g in gts} | {p.platform for p in preds}):
        pp = [p for p in preds if p.platform == plat]
        gg = [g for g in gts if g.platform == plat]
        per_platform[plat] = {
            "predictions": len(pp), "gt": len(gg),
            "class_agnostic": score(match(pp, gg, **gates), len(pp), len(gg)),
            "class_aware": score(match(pp, gg, class_aware=True, **gates), len(pp), len(gg)),
        }
    out["per_platform"] = per_platform
    return out


def split_by_source(preds: list[Prediction]) -> tuple[list[Prediction], list[Prediction]]:
    """(model predictions, GT-sourced predictions). GT-sourced detections are a
    self-check, never evidence of detector quality."""
    return ([p for p in preds if p.source != "ground-truth"],
            [p for p in preds if p.source == "ground-truth"])
