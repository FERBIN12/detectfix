# Bridge-Inspection Detection Pipeline

A one-way (forward-only) ground-station pipeline that turns a recorded drone +
ROV mission into a reviewed inspection report. It implements the flow in
[`PIPELINE.md`](PIPELINE.md) and the data contract in
[`DATA_INPUTS.md`](DATA_INPUTS.md); the system design is in
[`ARCHITECTURE.md`](ARCHITECTURE.md).

Design rule: **code enforces, models explain.** Ingest, indexing, retrace, LiDAR
depth estimation, correlation and severity are deterministic Python. The two LLM
agents (report + dashboard) only narrate and present the structured result, and
every report claim is validated against the detection data.

## Flow

```
1-2 ingest + decode + index  .mcap -> captures -> time/pose index   (indexing.py)
3   detect                   camera YOLO + sonar model              (detection.py)
4   retrace                  pull co-located recorded LiDAR/camera  (indexing.py, detection.py, lidar.py)
5   LiDAR depth estimation   measure depth/area -> geometry block   (lidar.py)
6   map/stitch               (location assignment stands in here)   (correlation.py)
7   correlate + severity     merge, locate, change-tag, rule severity (correlation.py, severity.py)
8   report agent (Claude)    draft narrative, validate vs. data     (agents.py)
9   dashboard agent (Claude) present report + metrics               (agents.py)
```

No step signals a vehicle. "Trigger on detection" is an internal retrace into
already-recorded data, keyed by timestamp + pose.

## Model slots

| Stage | Slot | Model |
|---|---|---|
| Detect (drone) | camera defect detector | [crack-detection-yolov8](https://github.com/sibghatullah78/crack-detection-yolov8) (YOLOv8-seg) |
| Detect (ROV) | sonar defect detector | [KD-YOLOX-ViT](https://github.com/remaro-network/KD-YOLOX-ViT) |
| LiDAR depth | point-cloud geometry | [pc_cr](https://github.com/yliu17lhr/pc_cr) / Open3D (no trained model) |
| Report + Dashboard | LLM agents | Claude API (default), local vLLM fallback |

## Run

Mock mode needs **no dependencies** (standard library only) and uses the demo
fixture:

```bash
python run.py                     # runs fixtures/demo_mission.json
python run.py --out out/demo.json # also write the full result
```

### Real ROS2 bag

`run_bag.py` runs the flow on an actual rosbag2 `.mcap` drone mission. It
decodes `/odom` (pose timeline), `/lidar/points` (retrace index) and
`/camera/rgb`, then runs the **real YOLOv8-seg defect model** over the
per-waypoint stills in `inspection_frames/`.

```bash
pip install mcap mcap-ros2-support zstandard numpy ultralytics huggingface_hub
# real model inference (weights auto-downloaded on first run):
python run_bag.py --bag ../data/run_20261024_123507 --detector model --out out/bag_model.json
# or use the bag's ground truth as the detection source instead:
python run_bag.py --bag ../data/run_20261024_123507 --detector gt --out out/bag_gt.json
```

The camera detector is **Kapuni Model A**
(`iloveass/kompon-damage-severity-detection`, YOLOv8s-seg), a multi-class
concrete-defect model (crack / spalling / rebar corrosion / severe distress /
microcrack) auto-downloaded into `weights/`. Its class names are mapped onto the
pipeline's drone schema in `detection.CAMERA_CLASS_MAP`.

> Windows note: `run_bag.py` sets `KMP_DUPLICATE_LIB_OK` / `OMP_NUM_THREADS=1`
> to avoid a torch<->numpy OpenMP clash that can otherwise hard-crash the
> process during point-cloud math.

### Annotated frames + video

`annotate_frames.py` runs the real model over each inspection frame, draws the
**segmentation masks + boxes + class/confidence labels**, writes annotated PNGs
to `web/data/annotated/`, and encodes a browser-playable **H.264 video**
(`web/data/feed.mp4`).

```bash
pip install imageio imageio-ffmpeg   # bundled ffmpeg for H.264 encoding
python annotate_frames.py --bag <BAG_DIR> --conf 0.15 --fps 3
# optional: a full-motion video from the whole /camera/rgb stream (slow, CPU)
python annotate_frames.py --bag <BAG_DIR> --rgb-video
```

### Web dashboard

A standalone, single-page web dashboard built for reviewers/judges. It presents
the **whole two-robot mission** — drone (ArduCopter) above the water line and
BlueROV2 (ArduSub) below it — entirely from the recorded bag, the maps, the
ground-truth defect list, the cooperation log and the pipeline result. **Every
number on the page is derived by `export_web.py`; nothing is hard-coded in the
UI.** Sections:

1. **Header** — Craigie Bridge · Charles River Dam, mission id, both platforms.
2. **KPI tiles** — model detections; 13 ground-truth defects (9 above / 4 below);
   4 waterline defects flagged → 4 inspected by the ROV; ROV navigation error vs.
   ground truth at the end (DVL dead-reckoning drift); ROV distance travelled;
   LiDAR points above / sonar points below; mission duration; drone waypoints.
3. **3D reconstruction (hero)** — a Three.js (pinned `three@0.160.0`) view of the
   combined map: height-coloured LiDAR above the translucent water plane (z = 0),
   amber sonar below it, the drone path (cyan), the ROV's DVL estimate (amber)
   and ground-truth track (thin white), ground-truth defect markers (red above /
   magenta below, hover for labels), and dashed lines from each drone-flagged
   waterline defect down to the ROV's dive point. The camera auto-frames the
   canal on load and slowly auto-orbits until you interact.
4. **Sensor feeds** — tabbed stills for Drone RGB (annotated), Thermal (simulated
   LWIR), ROV camera (turbid) and ROV sonar (multibeam fan), ~60 evenly spaced
   frames each, with play/scrub.
5. **Cooperation timeline** — drone flags vs. ROV dive/inspect events on the
   mission time axis.
6. **Detections table + AI report** — per-detection class/confidence/severity with
   evidence frame, the report with `[D-…]` citations (labelled honestly as
   deterministic when no LLM key is set), and a clickable
   Draft / Under review / Approved (PE sign-off) status control.
7. **Statewide triage** — top 10 Massachusetts bridges by underwater-inspection
   risk with reasons, plus the summary (5,252 bridges · 444 Poor · 778 needing
   underwater inspection · 123 scour critical), from FHWA NBI 2022.
8. **Footer** — data sources and honest-limits line.

```bash
# 1. run the pipeline on the drone+ROV bag (writes out/bag2_model.json)
python run_bag.py --bag <BAG_DIR> --detector model --out out/bag2_model.json
# 2. annotate the drone stills (optional; populates web/data/annotated/)
python annotate_frames.py --bag <BAG_DIR> --conf 0.15
# 3. build the dashboard bundle under web/data/ (clouds, paths, frames, triage)
python export_web.py --bag <BAG_DIR> --result out/bag2_model.json --frames
# 4. serve it (port 8099 by default; pass --bag to also serve original stills)
python serve.py --port 8099
# open http://127.0.0.1:8099
```

`export_web.py` is the single build command: it voxel-downsamples both maps to
≤ 250 k points each (compact Float32 binaries), decodes the bag once for the
drone/ROV paths, the ROV navigation error and distance, and the cooperation
events, extracts ~60 stills per sensor stream, copies the pipeline result, and
runs the statewide triage. `serve.py` is the single serve command. The full
bundle lives under `web/data/` (git-ignored). Reference screenshots are in
`docs/dashboard_v2_top.png` and `docs/dashboard_v2_full.png`.
Files: `serve.py`, `export_web.py`, `annotate_frames.py`, `web/*`.

LiDAR geometry here is measured off the mission's accumulated world map
(`lidar_map_world.npy`) as **local surface roughness** (PCA plane fit,
orthogonal-distance RMS in mm) around each defect. That is the honest metric the
map supports -- it is **not** a defect cavity depth, so no `area_mm2` is claimed.
Severity for GT-sourced detections comes from the bag's 1-3 rating, not the
roughness.

## Status / what is real vs. stubbed

| Piece | State |
|---|---|
| `.mcap` decode, time/pose index, retrace | real — runs on an actual rosbag2 bag |
| Drone camera detector (YOLOv8-seg) | **real model inference** (Kapuni Model A) |
| Sonar detector (ROV) | wired, **not exercised** — sonar imagery is recorded but no trained sonar model is run |
| ROV navigation (DVL vs. truth), cooperation | real — measured from `/rov/odom`, `/rov/truth/odom`, `/coop/tasks` |
| LiDAR geometry (surface roughness) | real, measured off the world-map point cloud |
| Correlation + rule-based severity | real |
| Report / dashboard LLM agents | **Claude by default**, falls back to deterministic text with no key |
| Web dashboard (3D + feeds + timeline + triage) | real, served locally |

Honest caveats worth knowing: the model was trained on real photos, so on
simulated imagery some detections are low-confidence (tune `--conf`); the bag
records sonar imagery but no trained sonar defect model is run, so the 4
underwater defects shown are ground truth surfaced via the cooperation log; the
thermal feed is simulated LWIR; and the LLM report is deterministic text unless
an `ANTHROPIC_API_KEY` is provided.

Live mode decodes a real `.mcap` and loads real weights:

```bash
pip install -r requirements.txt
cp .env.example .env              # add ANTHROPIC_API_KEY for the Claude agents
python run.py --mode live --mcap bags/mission1.mcap \
  --camera-weights weights/crack_yolov8_seg.pt \
  --sonar-weights weights/kd_yolox_vit.pth
```

## Agents / secrets

The report and dashboard agents default to the **Claude API**. Configure via
`.env` (see `.env.example`): `ANTHROPIC_API_KEY`, `CLAUDE_MODEL`, and
`AGENT_BACKEND` (`claude` or `local`). If Claude is unreachable the pipeline
falls back to the local model, then to deterministic rule-based text, so it
always produces a report. `.env` is gitignored.

## Tests

```bash
python -m unittest discover -s tests -v
```

## Layout

```
bridge-inspection/
  run.py              entry point for the demo/mock + generic live flow
  run_bag.py          run the flow on a real ROS2 .mcap drone mission
  annotate_frames.py  draw masks/boxes on frames + encode the H.264 video
  export_web.py       build the web data bundle (clouds + paths + frames + triage)
  serve.py            stdlib HTTP server for the web dashboard
  config.json         non-secret runtime settings
  .env.example        secret template (Claude key, backend)
  requirements.txt    live-mode deps (mock needs none)
  pipeline/
    schema.py         normalized detection record (the convergence point)
    config.py         .env + config.json loading
    indexing.py       .mcap decode, time/pose index, retrace
    detection.py      camera + sonar detectors (mock/live)
    rosbag.py         real rosbag2 decode, pose join, frame detection
    weights.py        auto-download the camera defect weights
    lidar.py          LiDAR geometry (surface roughness; Open3D + numpy/pure-Python)
    correlation.py    merge, dual-location, cross-cycle change tagging
    severity.py       rule-based condition states
    agents.py         Claude report + dashboard agents (validated)
    orchestrator.py   wires the one-way flow together
  fixtures/           offline demo bag, element map, prior cycle
  web/                dashboard UI (index.html, app.js, style.css)
  tests/              unittest smoke + behavior tests
```
