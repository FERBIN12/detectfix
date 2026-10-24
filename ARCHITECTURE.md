# AI-Assisted Bridge Inspection System — Design Summary

## The idea

MassDOT Aeronautics already flies drones for bridge/rail/road inspection, but the bottleneck has shifted from *capturing* data to *using* it: imagery, sonar scans, condition ratings, and GIS layers sit in separate systems, and FHWA's bridge data format is mid-transition (legacy NBI → SNBI, full compliance required by **March 15, 2028**), so any system has to tolerate two schemas at once.

This project proposes a ground-station-coordinated drone + ROV inspection system where:
- A **drone** (Holybro X500 V2, RGB camera + LiDAR) inspects the superstructure and surroundings.
- An **ROV** (sonar primary, camera to confirm) inspects the substructure underwater, directed in part by what the drone finds above.
- Defect **detection** is fully automated (YOLO-based computer vision — no manual frame review).
- A **human operator supervises the mission** (flight/dive plan, live feed, override) but does not do the detection work.
- An **agentic AI** correlates detections across sensors and inspection cycles and drafts the inspection report; it is constrained to the structured detection data (not raw imagery) and validated against it, so the model explains findings rather than inventing them.
- A **human inspector/PE signs off** on the draft before it becomes a record — this mirrors how real bridge inspection liability works, and it's a deliberate design choice, not a shortcut.

**Working assumption to state explicitly (and not overclaim):** a superstructure defect does *not* imply a substructure defect at the same spot — the two fail differently (cracking/spalling/corrosion above vs. scour/impact/biofouling below). What's valid is using a drone-geolocated defect as a **trigger to prioritize ROV re-inspection** of the connected substructure element, under a shared-risk-factor model (age, traffic, environmental exposure) rather than a causal one.

---

## System architecture

```
Drone (RGB camera primary, LiDAR for geometry/scale)     ROV (sonar primary locate, camera fine-ID)
        │ telemetry + video over radio                           │ video + sonar over tether
        ▼                                                         ▼
┌──────────────────────────── GROUND STATION INGEST ────────────────────────────┐
│  raw frames/scans + pose/GPS + timestamp → object storage + Postgres/PostGIS   │
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
┌───────────────────── DETECTION (automated, no human in the loop) ─────────────┐
│ YOLO models, separate weights per domain:                                     │
│   aerial camera → crack / spall / exposed rebar / corrosion stain             │
│   underwater sonar → anomalies (primary); ROV camera confirms when the water  │
│                      is clear enough (sonar-first, camera-confirm)            │
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
┌───────────────── MAPPING / LOCATION (single location, not per-frame) ─────────┐
│  accumulated LiDAR map (above water) + sonar map (below water), one world     │
│  frame; each defect is placed on it and assigned to the nearest element       │
│  near-planar walls → per-wall orthomosaic from the frames (planned)          │
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
┌───────────────── CORRELATION (deterministic code, not the LLM) ───────────────┐
│  merge near-duplicate detections by world position                            │
│  assign dual location: geodetic (lat/lon/elev) + structure-relative           │
│    (element ID, face, station, offset, elevation/depth)                       │
│  match against the prior inspection cycle's detection list                    │
│    → tag new / existing / worsening                                          │
│  compute severity from rules, not from the LLM's judgment                     │
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
┌───────────────────── AGENTIC AI — report drafting ────────────────────────────┐
│ Agent receives structured data only (class, confidence, location, severity,   │
│ change flag, element ID) — never raw images directly as "ground truth"        │
│ Tools: get_detections(mission_id), get_prior_inspection(element_id),          │
│        get_structure_metadata(bridge_id), get_evidence_frame(detection_id)    │
│ Drafts a per-element narrative, cites a detection ID for every claim          │
│ Backend validates draft against structured data; unsupported claims fall back │
│ to deterministic rule-based text (pattern used across 2026 inspection-LLM lit)│
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
          Inspector dashboard: draft → under_review → approved/rejected
                        (human sign-off gate — not optional)
                                     ▼
┌───────────────────────── STORAGE / DASHBOARD ─────────────────────────────────┐
│ approved report + full evidence chain + model version + approver ID → Postgres│
│ spatial dashboard: defect pins/heatmap on bridge model, review queue,         │
│ SNBI/NBI-schema export with legacy-vs-native provenance tagging              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware / sensing

| Platform | Primary defect sensor | Secondary sensor(s) | Role of secondary |
|---|---|---|---|
| Drone (Holybro X500 V2) | RGB camera → YOLO | LiDAR | 3D map, scale, surface geometry — **not** used for defect classification (too noisy/coarse for fine cracking at achievable standoff) |
| ROV (BlueROV2 Heavy) | Imaging sonar → YOLO | Camera | Confirms a sonar detection when a co-located frame is clear enough (measured RMS contrast); otherwise the detection stays sonar-only |

## Software / stack

| Layer | Choice | Why |
|---|---|---|
| Middleware | ROS2 (both vehicles) | One autonomy stack across sim and real hardware |
| Aerial GCS | QGroundControl / MAVSDK | |
| ROV control | ArduSub/BlueOS-style, tether + acoustic modem fallback | |
| Detection | YOLOv8 / YOLO11 (Ultralytics), domain-specific weights | Matches published results (CONBRID-YOLOv8, PolyU tuned YOLOv8) |
| Mapping | Accumulated LiDAR world map + per-wall orthomosaics (planned) | Locate defects once on a map instead of per frame |
| Underwater mapping | Accumulated sonar point map | Photomosaicking generally fails underwater (turbidity, low texture) |
| Agent framework | LLM agent, structured-data-only input, deterministic validation/fallback (read-only tool calls planned) | Mirrors the design converged on across 2026 inspection-report-LLM papers |
| Storage | Postgres/PostGIS (spatial + report records) + object storage for raw imagery/point clouds | |
| Schema layer | Versioned NBI↔SNBI crosswalk with provenance tagging (legacy-converted vs. SNBI-native) | FHWA itself says the two are indistinguishable without metadata during the transition window |
| Simulation | Isaac Sim (drone, bridge, LiDAR, simulated multibeam sonar + turbid-water camera) with ArduCopter / ArduSub SITL, recorded as ROS 2 bags | Both vehicles share one world frame and clock in the sim |

## Defect location schema (dual coordinate system)

| Field | Purpose |
|---|---|
| `lat, lon, elevation` | Dashboard map pin, GIS layer, SNBI geodetic export |
| `element_id, face, station, offset, elevation_or_depth` | Engineering record, matches AASHTO MBEI/SNBI element-level referencing, robust year-over-year comparison (structure-relative coordinates drift far less across inspection cycles than absolute GPS/mosaic registration does) |
| `detection_ids[]`, `model_version`, `confidence` | Audit trail — required since inspection reports carry legal/engineering weight |
| `change_status` (new / existing / worsening) | Computed in code by matching against the prior cycle's detections, never inferred by the LLM |
| `severity`, `severity_reason`, `recommended_action` | Set by rules in code; the reason is the rule trail shown to the inspector |

---

## Implemented vs. planned

| Component | State | Where / what is missing |
|---|---|---|
| One-way flow: decode, time+pose index, retrace | **implemented** | `rosbag.py`, `indexing.py`, `extract_rov.py` |
| Drone camera detection | **implemented** | Kapuni Model A (`detection.py`) |
| ROV sonar detection | **implemented** | Sonnet Model C, generic `sonar_anomaly` output (`rov.py`) |
| ROV camera confirm (clarity gate) | **implemented** | measured RMS contrast >= 0.03 |
| Geometry: surface roughness from LiDAR / sonar maps | **implemented** | `lidar.py`; no area, no cavity depth |
| Location: world position + nearest element | **implemented** | `correlation.py` |
| Correlation, change tagging, rule severity, recommended action | **implemented** | change tagging needs a prior-cycle detection list |
| Report agent with claim validation + deterministic fallback | **implemented** | `agents.py` (Claude / local / optional Kiro CLI) |
| Dashboard agent + Hyperion dashboard | **implemented** | `web/`, `serve.py` |
| Human sign-off gate | **implemented** | `review.py`: state machine, named reviewer, approval pinned to the report hash |
| Detection evaluation vs ground truth | **implemented** | `evaluation.py`, `eval_bag.py` |
| Platform health (mission + data quality) | **implemented** | `health.py`, `rov.py` |
| Hardware health (battery, motors, links, leaks) | **simulated** | `hardware.py`; bags record no hardware telemetry |
| Mask projection onto the LiDAR surface → `area_mm2` | planned | needs the camera field of view (no `camera_info` in the bags) |
| Per-wall orthomosaics, detect/merge on the mosaic | planned | needs the camera field of view |
| Duplicate tracking across frames (ByteTrack), SAHI tiling | planned | |
| Agent tool calls (`get_detections`, `get_prior_inspection`, …) | planned | the agent currently receives the structured data directly |
| Postgres/PostGIS storage + object storage | planned | results are JSON files today |
| NBI↔SNBI crosswalk export with provenance tags | planned | |
| Risk-based maintenance prioritization | planned | condition × age × traffic × location |

---

## Known gaps worth stating up front (strengthens, not weakens, the pitch)

1. **Drone↔ROV registration in the field.** In the sim both vehicles share one world frame. On a real bridge the ROV's DVL position must be tied to the drone's GPS frame; the pipeline reports DVL drift against truth (max 0.6 m on the drone+ROV mission) but does not correct it.
2. **No bridge-specific sonar model.** The sonar detector is a generic seabed-hazard model and found 1 of 4 underwater defects. Labelled sonar data of bridge substructure is needed.
3. **Dataset scarcity.** The camera model was trained on real photos and finds 3–4 of 9 simulated defects. Synthetic-to-real fine-tuning is needed in both directions.
4. **Bandwidth is a real deployment constraint** (tethered ROV + 4K drone feed). Plan for edge pre-filtering, not raw-stream-everything.
5. **No camera intrinsics in the recordings.** This blocks pixel-accurate defect placement, area measurement and orthomosaics until `camera_info` is recorded.

---

## Key sources

- Hu & Yee, [Integrated Framework for Bridge Crack Detection and Semantic BIM Model Generation](https://ascelibrary.org/doi/10.1061/9780784485163.020) — orthomosaic + segmentation, mIoU 0.787
- Sun et al. 2025, [Two-step rapid inspection of underwater concrete bridge structures combining sonar, camera, and deep learning](https://onlinelibrary.wiley.com/doi/10.1111/mice.13401) — 95.1% mAP sonar-locate/camera-confirm
- [FHWA Underwater Bridge Inspection Reference Manual (2024)](https://www.fhwa.dot.gov/bridge/nbis/pubs/nhi23027.pdf)
- [SNBI March 2022 publication (with Errata 1)](https://www.fhwa.dot.gov/bridge/snbi/snbi_march_2022_publication.pdf)
- [National Bridge Inventory — Based on the SNBI (FHWA)](https://www.fhwa.dot.gov/bridge/snbi.cfm)
- [CONBRID-YOLOv8](https://ijci.avestia.com/2026/PDF/006.pdf) — camera-based defect detection at UAV standoff
- [Agentic Federated LLMs at the Edge for Guardrail Inspection with YOLO-E](https://doi.org/10.1145/3773274.3777422) and [A Hybrid Vision-Language Architecture for Defect Reasoning and Report Generation](https://arxiv.org/pdf/2605.26533) — structured-data-to-LLM report pattern
- [A Preview of HoloOcean 2.0](https://arxiv.org/pdf/2510.06160) and [Towards Realistic 3D Sonar Simulation (Isaac Sim-based)](https://arxiv.org/pdf/2606.06130) — underwater sim options
- Chronological orthomosaic defect matching, JSCE 2024 — cross-cycle change detection precedent
