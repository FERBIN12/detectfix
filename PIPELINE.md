# End-to-End Detection Pipeline — One-Way Flow

This document defines the ground-station processing pipeline for AI-assisted
bridge inspection. It is a **one-way (forward-only) flow**: the drone and ROV
record all data continuously in a single pass, so the pipeline never signals a
vehicle to go back and re-capture. When a defect is detected, the system
**retraces into the already-recorded data** by timestamp and pose.

Design rule carried from `ARCHITECTURE.md`: **code enforces, models explain.**
Decoding, indexing, retrace, geometry, correlation, severity, evaluation and the
sign-off gate are deterministic Python. The LLM agents only draft the narrative
(report) and lay it out (dashboard); they never create detections, severities
or metrics.

Items marked *(planned)* are buildable with this data but not built yet. They
are listed again in `ARCHITECTURE.md`.

---

## Flow overview

```
1. INGEST      Drone + ROV record ALL streams continuously in one pass ->
               ROS 2 rosbag2 .mcap at the ground station.
                   |
                   v
2. DECODE      Read the .mcap (rosbag.py, extract_rov.py):
   + INDEX       drone: /odom pose timeline, /camera/rgb + /lidar/points timestamps,
                        per-waypoint inspection stills + sidecars + telemetry.csv
                 ROV:   /rov/odom (DVL/EKF pose + heading), /rov/truth/odom (drift only),
                        /rov/sonar/image, /rov/camera (+ measured visibility),
                        /rov/status, /coop/tasks
               Build a TIME + POSE INDEX over every stream (indexing.py).
               With no .mcap, decode from the frame sidecars instead.
                   |
                   v
3. DETECT      drone RGB  -> camera YOLOv8-seg (Kapuni Model A)
               ROV sonar  -> sonar YOLO11s (Sonnet Model C), run on the fan
                             unwarped to range x bearing. Code drops boxes over
                             25% of the frame, places each box in the world from
                             the ROV pose + heading + fan geometry, clusters hits
                             within 1.5 m, and keeps anomalies seen on >= 2 pings.
               Each detection carries class, confidence, box/mask, source
               frame/ping id, timestamp, pose and world position.
                   |
                   v
4. RETRACE     For each ROV sonar detection, the time+pose index resolves the
               co-located recorded ROV camera frame. It confirms the detection
               only if its RMS contrast >= 0.03 (clarity gate); otherwise the
               detection stays sonar-only. No re-capture -- the data already exists.
                   |
                   v
5. GEOMETRY    Crop the mission's accumulated world map around each defect's
               world position (drone: LiDAR map above water; ROV: sonar map
               below) and fit a local plane (PCA). Report the RMS deviation as
               local surface roughness in `geometry.depth_mm`. This is not a
               cavity depth, and no area is claimed.
               (planned) project the camera mask onto the LiDAR surface to
               measure area_mm2 -- needs the camera's field of view, which the
               bags do not record.
                   |
                   v
6. LOCATION    Assign each defect to the nearest bridge element by its world
               position (correlation.py). The sonar map (from the bag) and the
               combined above/below-water map are shown in Hyperion's 3D view.
               (planned) per-wall orthomosaics from the inspection frames
               projected onto each near-planar wall -- also needs the field of view.
                   |
                   v
7. CORRELATE   Merge near-duplicates (same class + platform within 0.5 m of world
   + SEVERITY  position); tag new / existing / worsening against a prior-cycle
               detection list when one is supplied; compute severity from RULES
               (class, geometry, worsening, confidence cap), recording the rule
               trail and a fixed recommended action. All deterministic.
                   |
                   v
8. REPORT      Report agent (Claude API by default) drafts a per-element
   (LLM)       narrative from STRUCTURED DATA ONLY, citing a detection_id for
               every claim. Code drops lines that cite an unknown id or state an
               uncited number; if nothing survives, deterministic text is used.
                   |
                   v
9. DASHBOARD   Dashboard agent lays out the validated report + metrics (order,
   (LLM)       grouping, labels; never values). Hyperion shows defects, report,
               3D map, frames, accuracy, real platform health and simulated
               hardware health.
                   |
                   v
   SIGN-OFF    Inspector review in Hyperion: draft -> under_review -> approved /
               rejected, named reviewer required, approval pinned to the report's
               hash (review.py). Not optional: the AI cannot change this state.

   EVALUATE    eval_bag.py scores detections against defects_gt.json: precision,
               recall, F1 per platform and class, plus a confidence sweep.
```

No step sends an arrow back to a vehicle.

---

## Why one-way works (retrace, not re-capture)

Both platforms record every stream continuously, so the data for every location
is already in the `.mcap`:

- **ROV / sonar -> camera:** a sonar detection's timestamp and pose resolve the
  recorded camera frame covering that region. The detection is confirmed when
  that frame is clear enough; otherwise it stays sonar-only.
- **Drone / camera -> map:** the defect's world position selects the matching
  patch of the accumulated LiDAR map for the geometry measurement.

This makes the pipeline deterministic and replayable from a single `.mcap`. It
depends on all sensors sharing a clock and being calibrated to each other.

---

## Model slots

| Stage | Slot | Model | Note |
|---|---|---|---|
| 3 Detect (drone) | camera defect detector | Kapuni Model A (YOLOv8s-seg) | concrete defects; trained on real photos |
| 3 Detect (ROV) | sonar detector | Sonnet Model C (YOLO11s) | generic seabed-hazard model; output is `sonar_anomaly` only |
| 5 Geometry | surface fit | none (numpy PCA) | roughness only |
| 8 Report | LLM report agent | Claude API (default) | local model fallback; optional Kiro CLI |
| 9 Dashboard | LLM dashboard agent | Claude API (default) | presents only |

---

## Related docs

- `ARCHITECTURE.md`: system design, implemented vs. planned, known gaps
- `DATA_INPUTS.md`: what the drone and ROV must provide
- `README.md`: how to run, results, status
