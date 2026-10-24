"""Platform health + data quality metrics (dashboard Tab 2).

Computes real drone health from the mission telemetry (inspection_frames/
telemetry.csv): utilization, GNSS fix quality, HDOP, LiDAR point-rate, per-
element dwell time, pose coverage. The ROV has no data in a drone-only bag, so
its block is explicit FILLER, flagged `is_filler=True`, never presented as real.

Code enforces: every number here is computed from the recorded telemetry, not
inferred by the LLM.
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional

# Mission phases: anything not "transit" is active inspection of an element.
TRANSIT = "transit"


def _f(v, default=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def drone_health(bag_dir: str) -> Optional[dict]:
    """Parse telemetry.csv into a drone health + data-quality report."""
    csv_path = Path(bag_dir) / "inspection_frames" / "telemetry.csv"
    if not csv_path.exists():
        return None
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8-sig")))
    if not rows:
        return None

    n = len(rows)
    t0, t1 = _f(rows[0].get("t")), _f(rows[-1].get("t"))
    duration_s = max(t1 - t0, 0.0)
    dt = duration_s / (n - 1) if n > 1 else 0.1   # telemetry step (~0.1 s)

    # Utilization: steps on an element vs. transit.
    per_element: dict[str, int] = {}
    inspecting = 0
    sats, hdops, lidar_pts = [], [], []
    rtk_fixed = 0
    armed = 0
    for r in rows:
        el = (r.get("element") or "").strip()
        if el and el != TRANSIT:
            inspecting += 1
            per_element[el] = per_element.get(el, 0) + 1
        sats.append(_f(r.get("gnss_sats")))
        hdops.append(_f(r.get("hdop")))
        lidar_pts.append(_f(r.get("lidar_pts")))
        fix = (r.get("fix") or "")
        if "RTK FIXED" in fix:
            rtk_fixed += 1
        if "ARMED" in fix:
            armed += 1

    def avg(xs):
        xs = [x for x in xs if x]
        return round(sum(xs) / len(xs), 2) if xs else 0.0

    def pct(count):
        return round(100.0 * count / n, 1) if n else 0.0

    # LiDAR point-rate stability: how often the per-step count drops well below
    # the median (a proxy for sensor dropouts / thin returns).
    lp = sorted(x for x in lidar_pts if x)
    median_lp = lp[len(lp) // 2] if lp else 0.0
    low_returns = sum(1 for x in lidar_pts if x and x < 0.5 * median_lp)

    elements = [
        {"element": el, "steps": c,
         "seconds": round(c * dt, 1),
         "share_pct": pct(c)}
        for el, c in sorted(per_element.items(), key=lambda kv: -kv[1])
    ]

    return {
        "platform": "drone",
        "model": "Holybro X500 V2 (RGB + Ouster OS0-128 LiDAR)",
        "is_filler": False,
        "mission": {
            "duration_s": round(duration_s, 1),
            "telemetry_hz": round(1.0 / dt, 1) if dt else None,
            "samples": n,
        },
        "utilization": {
            "inspecting_pct": pct(inspecting),
            "transit_pct": pct(n - inspecting),
            "armed_pct": pct(armed),
            "per_element": elements,
        },
        "gnss": {
            "rtk_fixed_pct": pct(rtk_fixed),
            "avg_sats": avg(sats),
            "avg_hdop": avg(hdops),
            "quality": _gnss_grade(pct(rtk_fixed), avg(hdops)),
        },
        "lidar": {
            "avg_points_per_scan": int(avg(lidar_pts)),
            "median_points_per_scan": int(median_lp),
            "low_return_steps": low_returns,
            "low_return_pct": pct(low_returns),
        },
        "data_quality": _data_quality(pct(rtk_fixed), avg(hdops), low_returns, n),
    }


def _gnss_grade(rtk_pct, hdop):
    if rtk_pct >= 95 and hdop <= 1.5:
        return "excellent"
    if rtk_pct >= 80 and hdop <= 2.5:
        return "good"
    if rtk_pct >= 50:
        return "fair"
    return "poor"


def _data_quality(rtk_pct, hdop, low_returns, n):
    issues = []
    if rtk_pct < 95:
        issues.append(f"RTK fix held {rtk_pct}% of the mission")
    if hdop > 1.5:
        issues.append(f"mean HDOP {hdop} above 1.5")
    if low_returns > 0.1 * n:
        issues.append(f"{low_returns} steps with low LiDAR returns")
    grade = "high" if not issues else ("moderate" if len(issues) == 1 else "degraded")
    return {"grade": grade, "issues": issues}


def rov_health_filler() -> dict:
    """Explicit placeholder for the ROV. This bag is drone-only, so there is no
    real ROV telemetry; these values are clearly marked filler and must be shown
    as such in the UI, never as measured data."""
    return {
        "platform": "rov",
        "model": "BlueROV-class (imaging sonar + camera) — not flown this mission",
        "is_filler": True,
        "mission": {"duration_s": 0.0, "telemetry_hz": None, "samples": 0},
        "utilization": {
            "inspecting_pct": 0.0, "transit_pct": 0.0, "armed_pct": 0.0,
            "per_element": [
                {"element": "PIER-01 (filler)", "steps": 0, "seconds": 0.0, "share_pct": 0.0},
                {"element": "PIER-02 (filler)", "steps": 0, "seconds": 0.0, "share_pct": 0.0},
            ],
        },
        "sonar": {"avg_range_m": None, "coverage_pct": None, "note": "filler — no sonar stream in this bag"},
        "camera": {"avg_visibility": None, "confirmed_pct": None,
                   "note": "filler — clarity-gated confirms would appear here"},
        "data_quality": {"grade": "n/a", "issues": ["no ROV deployed in this mission (filler values)"]},
    }


def build_health(bag_dir: str) -> dict:
    """Both platforms' health for the dashboard. Drone is real; ROV is filler."""
    drone = drone_health(bag_dir)
    return {
        "drone": drone,
        "rov": rov_health_filler(),
        "has_real_drone": drone is not None,
    }
