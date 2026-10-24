"""CORRELATE (pipeline step 7, first half).

Deterministic, not the LLM:
  - merge near-duplicate detections (same class, co-located)
  - assign dual location (geodetic from GPS + structure-relative from element map)
  - match against the prior inspection cycle -> new / existing / worsening

Severity is computed separately in severity.py and attached after this stage.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

from .schema import Detection

# Two detections of the same class within this 3D distance (metres) are treated
# as the same physical defect seen twice.
MERGE_DIST_M = 0.5
# Match radius against a prior-cycle defect (structure-relative is more stable,
# but we fall back to geodetic distance when element refs are absent).
PRIOR_MATCH_M = 1.0


def _dist(a: Detection, b: Detection) -> float:
    return math.sqrt((a.pose.x - b.pose.x) ** 2
                     + (a.pose.y - b.pose.y) ** 2
                     + (a.pose.z - b.pose.z) ** 2)


def merge_duplicates(detections: list[Detection]) -> list[Detection]:
    """Collapse near-duplicate detections of the same class into one record,
    keeping the highest-confidence instance and unioning evidence frames."""
    kept: list[Detection] = []
    for det in sorted(detections, key=lambda d: -d.confidence):
        dup = None
        for k in kept:
            if k.cls == det.cls and k.platform == det.platform and _dist(k, det) <= MERGE_DIST_M:
                dup = k
                break
        if dup is None:
            kept.append(det)
        else:
            for fid in det.evidence_frame_ids:
                if fid not in dup.evidence_frame_ids:
                    dup.evidence_frame_ids.append(fid)
            # Keep the deeper/larger geometry if the duplicate measured more.
            if det.geometry.is_measured() and not dup.geometry.is_measured():
                dup.geometry = det.geometry
    return kept


def assign_locations(detections: list[Detection], element_map: Optional[dict] = None) -> None:
    """Fill the structure-relative location from an element map.

    The element map keys an element_id/face/station to a geodetic+pose region;
    here we attach the nearest element to each detection. Geodetic coordinates
    are assumed already present from the GPS tag captured at detection time.
    element_map shape: {"elements": [{"element_id","face","x","y","z",
    "station","offset"}]}
    """
    if not element_map:
        return
    elements = element_map.get("elements", [])
    for det in detections:
        best, best_d = None, float("inf")
        for el in elements:
            d = math.sqrt((det.pose.x - el.get("x", 0)) ** 2
                          + (det.pose.y - el.get("y", 0)) ** 2
                          + (det.pose.z - el.get("z", 0)) ** 2)
            if d < best_d:
                best, best_d = el, d
        if best is not None:
            det.location.structure.element_id = best.get("element_id", "")
            det.location.structure.face = best.get("face", "")
            det.location.structure.station = best.get("station")
            det.location.structure.offset = best.get("offset")
            if det.platform == "rov":
                det.location.structure.elevation_or_depth = det.location.geodetic.elevation


def load_prior(prior_dir: Optional[str]) -> list[dict]:
    """Load the prior inspection cycle's detections (JSON) for change matching."""
    if not prior_dir:
        return []
    p = Path(prior_dir) / "detections.json"
    if not p.exists():
        return []
    return json.loads(p.read_text(encoding="utf-8-sig")).get("detections", [])


def tag_change_status(detections: list[Detection], prior: list[dict]) -> None:
    """Set change_status by matching each detection to the prior cycle.

    new       : no prior defect of this class within PRIOR_MATCH_M
    existing  : matched, geometry comparable or smaller
    worsening : matched and measured geometry grew beyond tolerance
    """
    for det in detections:
        match = _nearest_prior(det, prior)
        if match is None:
            det.change_status = "new"
            continue
        det.change_status = "worsening" if _has_worsened(det, match) else "existing"


def _nearest_prior(det: Detection, prior: list[dict]) -> Optional[dict]:
    best, best_d = None, PRIOR_MATCH_M
    for pr in prior:
        if pr.get("class") != det.cls or pr.get("platform") != det.platform:
            continue
        pp = pr.get("pose", {})
        d = math.sqrt((det.pose.x - pp.get("x", 0)) ** 2
                      + (det.pose.y - pp.get("y", 0)) ** 2
                      + (det.pose.z - pp.get("z", 0)) ** 2)
        if d <= best_d:
            best, best_d = pr, d
    return best


def _has_worsened(det: Detection, prior: dict) -> bool:
    """Geometry-based worsening: >20% growth in depth or area counts as worse."""
    pg = prior.get("geometry") or {}
    for field_name, cur in (("depth_mm", det.geometry.depth_mm), ("area_mm2", det.geometry.area_mm2)):
        prev = pg.get(field_name)
        if cur is not None and prev:
            if cur > prev * 1.2:
                return True
    return False


def run_correlation(detections: list[Detection], element_map: Optional[dict],
                    prior: list[dict]) -> list[Detection]:
    """Step 7 (first half): merge, locate, tag change. Severity comes after."""
    merged = merge_duplicates(detections)
    assign_locations(merged, element_map)
    tag_change_status(merged, prior)
    return merged
