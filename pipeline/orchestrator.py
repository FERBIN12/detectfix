"""The one-way pipeline, wired end to end (PIPELINE.md steps 1-9).

    ingest+decode+index -> detect -> retrace (camera-confirm + lidar) ->
    lidar depth -> correlate -> severity -> report agent -> dashboard agent

No step signals a vehicle. Every stage is deterministic except the two Claude
agents, which only narrate/present the structured result.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from . import agents, correlation, detection, indexing, lidar, severity
from .schema import Detection


def _load_element_map(data_dir: Optional[str]) -> Optional[dict]:
    if not data_dir:
        return None
    p = Path(data_dir) / "element_map.json"
    if not p.exists():
        return None
    return json.loads(p.read_text(encoding="utf-8-sig"))


def compute_metrics(detections: list[Detection]) -> dict:
    sev = {s: 0 for s in severity.STATES}
    for d in detections:
        if d.severity in sev:
            sev[d.severity] += 1
    return {
        "defect_count": len(detections),
        "drone": sum(1 for d in detections if d.platform == "drone"),
        "rov": sum(1 for d in detections if d.platform == "rov"),
        "sonar_only": sum(1 for d in detections if d.platform == "rov" and d.confirmed is False),
        "worsening": sum(1 for d in detections if d.change_status == "worsening"),
        "new": sum(1 for d in detections if d.change_status == "new"),
        **sev,
    }


def run_pipeline(mission_id: str, mcap_path: str, *,
                 camera_weights: Optional[str] = None,
                 sonar_weights: Optional[str] = None,
                 prior_dir: Optional[str] = None,
                 data_dir: Optional[str] = None,
                 max_dt_s: float = 0.5,
                 max_pose_m: float = 2.0) -> dict:
    """Execute the full one-way flow and return the report + dashboard + records."""
    # 1-2. INGEST + DECODE + INDEX
    captures = indexing.decode_mcap(mcap_path)
    index = indexing.build_index(captures, max_dt_s=max_dt_s, max_pose_m=max_pose_m)

    # 3. DETECT
    camera = detection.CameraDetector(weights=camera_weights)
    sonar = detection.SonarDetector(weights=sonar_weights)
    detections = detection.run_detection(mission_id, index, captures, camera, sonar)

    # 4. RETRACE: ROV camera-confirm + (below) LiDAR cloud lookup
    detection.confirm_rov_detections(index, detections)

    # 5. LiDAR DEPTH ESTIMATION (fills geometry on drone detections)
    lidar.run_lidar_depth(index, detections)

    # 6. MAP / STITCH is where orthomosaic/mesh/sonar-3D would refine locations;
    #    represented here by the element-map location assignment in CORRELATE.

    # 7. CORRELATE + SEVERITY
    element_map = _load_element_map(data_dir)
    prior = correlation.load_prior(prior_dir)
    detections = correlation.run_correlation(detections, element_map, prior)
    severity.run_severity(detections)

    # metrics for the dashboard (computed, not inferred)
    metrics = compute_metrics(detections)

    # 8. REPORT (Claude agent, validated against structured data)
    report = agents.draft_report(detections)

    # 9. DASHBOARD (Claude agent, presentation only)
    dashboard = agents.build_dashboard(report, metrics)

    return {
        "mission_id": mission_id,
        "detections": [d.to_dict() for d in detections],
        "metrics": metrics,
        "report": report,
        "dashboard": dashboard,
        "index_counts": {k: index.count(k) for k in indexing.STREAM_KINDS},
    }
