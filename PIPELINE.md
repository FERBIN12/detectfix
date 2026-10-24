# End-to-End Detection Pipeline — One-Way Flow

This document defines the ground-station processing pipeline for AI-assisted
bridge inspection. It is a **one-way (forward-only) flow**: the drone and ROV
record all data continuously in a single pass, so the pipeline never signals a
vehicle to go back and re-capture. When a defect is detected, the system
**retraces into the already-recorded data** by timestamp and pose to pull the
co-located LiDAR cloud / camera frame it needs.

Design rule carried from `ARCHITECTURE.md`: **code enforces, models explain.**
Ingest, decoding, indexing, retrace, LiDAR depth estimation, correlation, and
severity are deterministic Python. The LLM agents only draft narrative (report)
and present it (dashboard); they never invent detections or severity.

---

## Flow overview

```
1. INGEST      Drone + ROV record ALL data continuously (single pass) →
               delivered to the ground station as rosbag .mcap
                   │
                   ▼
2. DECODE      Read/decode .mcap (ROS2 messages → RGB frames, LiDAR clouds,
   + INDEX     sonar frames, pose, GPS) + preprocessing.
               Build a TIME + POSE INDEX over every stream so any (timestamp,
               pose) can resolve the co-located data from the other sensors.
                   │
                   ▼
3. DETECT      Decoded data → detection models:
                 drone RGB  → camera YOLO        (crack-detection-yolov8 slot)
                 ROV sonar  → sonar model         (KD-YOLOX-ViT slot)
               Each detection carries: class, confidence, 2D mask/box,
               source frame/scan id, timestamp, pose.
                   │
                   ▼
4. RETRACE     For each defect, look up co-located recorded data via the
               time+pose index (NO re-capture — the data already exists):
                 drone → recorded LiDAR cloud covering the defect region
                 ROV   → recorded camera frame covering the defect region
                         (clarity-gated: skip if turbidity defeats the camera)
                   │
                   ▼
5. LiDAR       Point-cloud geometric processing (pc_cr / Open3D slot):
   DEPTH         • project 2D defect mask into the cloud (intrinsics+extrinsics)
   ESTIMATION    • isolate defect-region points
                 • fit local surface → measure depth_mm, area_mm2, deviation
                 • (cross-cycle) register vs. prior cloud → deflection / change
               → fills the `geometry` block on the detection record.
                   │
                   ▼
6. MAP /       Orthomosaic (near-planar faces) / textured 3D mesh (non-planar) /
   STITCH      sonar 3D reconstruction (underwater). Georeferenced via GCPs /
               physical fiducials.
                   │
                   ▼
7. CORRELATE   Merge near-duplicate detections; assign dual location (geodetic +
   + SEVERITY  structure-relative); cross-cycle feature match → new/existing/
               worsening; compute severity from RULES using the measured
               geometry from step 5. All deterministic — not the LLM.
                   │
                   ▼
8. REPORT      AI agent (Claude API, default) drafts a per-element narrative from
   (Claude)    STRUCTURED DATA ONLY, citing a detection_id for every claim.
               Backend validates the draft against the structured data;
               unsupported claims fall back to deterministic rule-based text.
                   │
                   ▼
9. DASHBOARD   A second LLM agent (Claude API, default) handles dashboard
   (Claude)    updates — it uses model judgment to decide what/how to present
               the report plus metrics (hardware health, bridge health).
                   │
                   ▼
          Inspector dashboard: draft → under_review → approved/rejected
                      (human sign-off gate — not optional)
```

No step sends an arrow back to a vehicle. The "alert on detection" is an
**internal event** that kicks off retrace + LiDAR depth estimation for that
defect — never an outbound command.

---

## Why one-way works (retrace, not re-capture)

Because both platforms record every stream continuously, the data for every
location is already in the `.mcap`. A detection does not need the vehicle to
return; it only needs to find the recorded data that covers the same spot:

- **Drone / crack → LiDAR:** the camera frame that flagged the crack has a
  timestamp and pose. The time+pose index resolves the LiDAR cloud captured at
  that same instant/region. Depth estimation runs on that existing cloud.
- **ROV / sonar → camera:** the sonar detection's timestamp and pose resolve the
  recorded camera frame covering that region, used to confirm (when water
  clarity allows; otherwise the defect stays sonar-only).

This makes the whole pipeline **deterministic and replayable** from a single
`.mcap` and fits "code enforces." Its one hard requirement is tight cross-sensor
time-sync + spatial calibration (see the sync/extrinsics Known Gap in
`ARCHITECTURE.md`): retrace-by-timestamp-and-pose only works if all sensors
share a clock and are calibrated to one another.

---

## Model slots in the pipeline

| Stage | Slot | Candidate | Integration note |
|---|---|---|---|
| 3 Detect (drone) | camera defect detector | `crack-detection-yolov8` (YOLOv8-seg) | template to fine-tune to multi-class (crack/spall/exposed_rebar/corrosion_stain) |
| 3 Detect (ROV) | sonar defect detector | `KD-YOLOX-ViT` | ready weights (SWDD/wall) as pretrain base; fine-tune to pier/substructure defect classes + sonar type |
| 5 LiDAR depth | point-cloud geometry | `pc_cr` / Open3D | no trained model; change-based measurement + single-mission surface fit |
| 8 Report | LLM report agent | Claude API (default) | model-agnostic interface; local model remains a fallback |
| 9 Dashboard | LLM dashboard agent | Claude API (default) | true LLM agent; decides what/how to present |

---

## LLM agents — Claude API

Both the report agent (step 8) and the dashboard agent (step 9) default to the
**Claude API**, replacing the repo's local Qwen/vLLM setup. The agent interface
is kept **model-agnostic** so a local model remains a fallback option, but Claude
is the intended default.

- This introduces an **external API dependency** (network + API key) that the
  current self-hosted setup does not have — a deliberate deviation.
- Both agents remain constrained by "code enforces, models explain": the report
  agent reads structured data only and is validated against it; the dashboard
  agent presents already-computed metrics and the validated report.

---

## Related docs

- `ARCHITECTURE.md` — full system design, hardware/sensing, schema, known gaps
- `DATA_INPUTS.md` — what the drone and ROV must provide (incl. continuous
  recording + timestamp/pose indexing that make retrace possible)
