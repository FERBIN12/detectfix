# Product

A forward-only ground-station pipeline that turns a recorded drone + ROV bridge-inspection mission (ROS 2 rosbag2 `.mcap`) into a reviewed inspection report, shown in the **Hyperion** web dashboard. Challenge track: Robotics, Safer Inspection of Critical Infrastructure.

## What it does
- Detects defects on drone camera imagery (YOLOv8-seg) and ROV sonar (YOLO11s), then confirms sonar hits with a co-located ROV camera frame when the water is clear enough.
- Places each defect in a shared world frame, assigns it to a bridge element, merges duplicates, tags change vs the prior cycle (new / existing / worsening), and sets a rule-based severity (good / fair / poor / severe) with a rule trail and a recommended action.
- An LLM agent drafts the report and a second one lays out the dashboard. Code validates every claim against the detection data.
- A human inspector signs off: draft → under_review → approved / rejected. Each approval is pinned to a hash of the report it approved.
- Scores detections against ground truth (precision / recall).

## Core principle: code enforces, models explain
Facts (severity, change status, class, geometry, location, metrics, hardware status, sign-off state) come only from deterministic code. LLMs only narrate. See `code-enforces-models-explain.md`.

## Honest scope
- Data is simulated (Isaac Sim + ArduPilot SITL, Craigie Drawbridge, Boston). Two missions: `run_20261024_123507` (drone only) and `run_20261024_134808` (drone + BlueROV2).
- "Trigger on detection" means retracing already-recorded data by timestamp + pose. No step commands a vehicle.
- The sonar model is generic (seabed hazards), so its output is always `sonar_anomaly`.
- Hardware health is simulated and must stay labelled as such.
- Planned, not built: mask area (`area_mm2`), orthomosaics, ByteTrack/SAHI, agent tool calls, Postgres/PostGIS, NBI↔SNBI export. Don't describe these as implemented. The "Implemented vs. planned" table in `ARCHITECTURE.md` is the source of truth.
