# AI-Assisted Bridge Inspection System — Design Summary

## The idea

MassDOT Aeronautics already flies drones for bridge/rail/road inspection, but the bottleneck has shifted from *capturing* data to *using* it: imagery, sonar scans, condition ratings, and GIS layers sit in separate systems, and FHWA's bridge data format is mid-transition (legacy NBI → SNBI, full compliance required by **March 15, 2028**), so any system has to tolerate two schemas at once.

This project proposes a ground-station-coordinated drone + ROV inspection system where:
- A **drone** (Holybro X500 V2, RGB camera + LiDAR) inspects the superstructure and surroundings.
- An **ROV** (sonar + camera) inspects the substructure underwater, directed in part by what the drone finds above.
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
│ YOLOv8/v11, separate fine-tuned weights per domain:                           │
│   aerial  → crack / spall / exposed rebar / corrosion stain                   │
│   underwater → pier damage / visible biofouling (camera-based, sonar-gated)   │
│ two-pass: fast onboard edge pass (live ops) + full-res ground-station batch   │
│ pass (final record)                                                           │
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
┌───────────────── MAPPING / STITCHING (single location, not per-frame) ────────┐
│  near-planar faces (deck, abutments)   → orthomosaic (OpenDroneMap/WebODM),   │
│                                            detect-once on the mosaic           │
│  non-planar (piers, bearings, underside)→ textured 3D mesh (SfM + LiDAR),     │
│                                            per-frame detections projected on  │
│  underwater (turbidity defeats photomosaic)→ sonar-based 3D pier reconstruction│
│  georeferenced via GCPs / physical fiducials on the structure                 │
└───────────────────────────────────┬───────────────────────────────────────────┘
                                     ▼
┌───────────────── CORRELATION (deterministic code, not the LLM) ───────────────┐
│  merge near-duplicate detections in mosaic/mesh coordinates                    │
│  assign dual location: geodetic (lat/lon/elev) + structure-relative           │
│    (element ID, face, station, offset, elevation/depth)                       │
│  match against prior inspection cycle's mosaic/mesh (feature matching)        │
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
| Drone (Holybro X500 V2) | RGB camera → YOLO | LiDAR | 3D geometry, scale, SLAM pose, coverage/collision — **not** used for defect classification (too noisy/coarse for fine cracking at achievable standoff) |
| ROV | Camera → YOLO (when visibility allows) | Sonar | Macro-localizes pier/candidate defect region in turbid water before camera confirms (sonar-first, camera-confirm) |

## Software / stack

| Layer | Choice | Why |
|---|---|---|
| Middleware | ROS2 (both vehicles) | One autonomy stack across sim and real hardware |
| Aerial GCS | QGroundControl / MAVSDK | |
| ROV control | ArduSub/BlueOS-style, tether + acoustic modem fallback | |
| Detection | YOLOv8/v11, domain-specific fine-tuned weights, SAHI tiling for thin/small defects | Matches published results (CONBRID-YOLOv8, PolyU tuned YOLOv8) |
| Tracking/mapping | ByteTrack + OpenDroneMap/WebODM (orthomosaic, GCP georeferencing); textured mesh for non-planar members | Avoids counting the same defect twice; avoids parallax error from flat-mosaicing curved geometry |
| Underwater mapping | Sonar-based 3D pier reconstruction | Photomosaicking generally fails underwater (turbidity, low texture) |
| Agent framework | Tool-calling LLM agent, read-only tools, structured-data-only input, deterministic validation/fallback | Mirrors the design converged on across 2026 inspection-report-LLM papers; also the exact pattern already proven in this repo's `supervisor/` + `dashboard/` (Hyperion) — reusable skeleton for a hackathon timeline |
| Storage | Postgres/PostGIS (spatial + report records) + object storage for raw imagery/point clouds | |
| Schema layer | Versioned NBI↔SNBI crosswalk with provenance tagging (legacy-converted vs. SNBI-native) | FHWA itself says the two are indistinguishable without metadata during the transition window |
| Simulation | Isaac Sim (drone + bridge, RTX LiDAR) bridged via ROS2 to HoloOcean 2.0 or an Isaac-Sim-based sonar framework (ROV side) | Isaac Sim has no native underwater/sonar simulation — needs a second engine sharing the same topics |

## Defect location schema (dual coordinate system)

| Field | Purpose |
|---|---|
| `lat, lon, elevation` | Dashboard map pin, GIS layer, SNBI geodetic export |
| `element_id, face, station, offset, elevation_or_depth` | Engineering record, matches AASHTO MBEI/SNBI element-level referencing, robust year-over-year comparison (structure-relative coordinates drift far less across inspection cycles than absolute GPS/mosaic registration does) |
| `detection_ids[]`, `model_version`, `confidence` | Audit trail — required since inspection reports carry legal/engineering weight |
| `change_status` (new / existing / worsening) | Computed in code from cross-cycle mosaic/mesh feature matching, never inferred by the LLM |

---

## Known gaps worth stating up front (strengthens, not weakens, the pitch)

1. **Drone↔ROV geo-registration is unsolved without physical fiducials on the piers** — needed to tie the ROV's dead-reckoned position to the drone's GPS frame.
2. **Biofouling-via-sonar has no literature precedent** — existing biofouling classifiers are all optical; budget for original data collection here.
3. **Dataset scarcity is real** across nearly every cited paper — plan on Isaac Sim synthetic data (rendered cracks/spalling/biofouling textures) for pretraining, fine-tuned on a small real set.
4. **Bandwidth is a real deployment constraint** (tethered ROV + 4K drone feed) — plan for edge pre-filtering, not raw-stream-everything.
5. **Mosaic/mesh stitching has its own error sources** (seams, parallax on non-planar surfaces) — the final proximity-merge pass is still required, just much lighter than full frame-by-frame triangulation.

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
