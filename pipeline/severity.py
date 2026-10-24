"""SEVERITY (pipeline step 7, second half).

Severity is computed from RULES, never from the LLM's judgment. The thresholds
live here in code, not in any prompt -- the same discipline as the supervisor's
detector thresholds. Rules are loosely modelled on AASHTO MBEI condition states
(good / fair / poor / severe), driven by class, measured geometry, and change.
"""
from __future__ import annotations

from .schema import Detection

# Condition-state ladder, worst last.
STATES = ("good", "fair", "poor", "severe")

# Per-class geometry thresholds (mm / mm2) for the depth/area that pushes a
# defect from fair -> poor -> severe. None means the dimension is not used for
# that class. These are deliberate, reviewable constants.
DEPTH_MM = {
    "spall":        {"poor": 25.0, "severe": 50.0},
    "scour":        {"poor": 150.0, "severe": 400.0},
    "section_loss": {"poor": 5.0, "severe": 15.0},
    "pier_damage":  {"poor": 30.0, "severe": 80.0},
}
AREA_MM2 = {
    "spall":        {"poor": 10000.0, "severe": 40000.0},
    "corrosion_stain": {"poor": 20000.0, "severe": 80000.0},
}

# Classes that are serious on presence alone (any confident detection is at
# least "poor"): exposed rebar means the protective cover is already gone.
PRESENCE_POOR = {"exposed_rebar"}

# Surface/cosmetic classes: a confident detection is only "fair" unless a
# measured dimension or worsening pushes it up. Efflorescence and rust streaking
# are early indicators, not structural loss on their own.
SURFACE_CLASSES = {"efflorescence", "rust_streak", "corrosion_stain"}


def _rank(state: str) -> int:
    return STATES.index(state)


def _worst(*states: str) -> str:
    return max(states, key=_rank)


# When a detection carries an authoritative severity rating (e.g. a 1-3 scale
# from ground truth or an inspector), map it directly rather than inferring from
# LiDAR surface roughness, which is not a cavity depth.
GT_SEVERITY_STATE = {1: "fair", 2: "poor", 3: "severe"}


def classify(det: Detection) -> str:
    """Return a condition state for one detection from rules only."""
    gt = getattr(det, "_gt_severity", 0)
    if gt in GT_SEVERITY_STATE:
        state = GT_SEVERITY_STATE[gt]
        if det.change_status == "worsening":
            state = STATES[min(_rank(state) + 1, len(STATES) - 1)]
        return state

    state = "fair"  # a confident defect detection is at least "fair"

    if det.cls in PRESENCE_POOR:
        state = _worst(state, "poor")

    depth = det.geometry.depth_mm
    if depth is not None and det.cls in DEPTH_MM:
        th = DEPTH_MM[det.cls]
        if depth >= th["severe"]:
            state = _worst(state, "severe")
        elif depth >= th["poor"]:
            state = _worst(state, "poor")

    area = det.geometry.area_mm2
    if area is not None and det.cls in AREA_MM2:
        th = AREA_MM2[det.cls]
        if area >= th["severe"]:
            state = _worst(state, "severe")
        elif area >= th["poor"]:
            state = _worst(state, "poor")

    # Worsening since the last cycle bumps severity one step (capped at severe).
    if det.change_status == "worsening":
        state = STATES[min(_rank(state) + 1, len(STATES) - 1)]

    # An unconfirmed ROV sonar hit (camera could not verify in turbid water)
    # is not downgraded -- sonar is the primary underwater detector -- but a
    # very low-confidence detection stays "fair" regardless of geometry noise.
    if det.confidence < 0.4 and _rank(state) > _rank("fair"):
        state = "fair"

    return state


def run_severity(detections: list[Detection]) -> None:
    """Step 7 (second half): attach a rule-based severity to each detection."""
    for det in detections:
        det.severity = classify(det)
