# Bridge-Inspection Detection Pipeline + Hyperion dashboard

**Challenge track:** Robotics — Safer Inspection of Critical Infrastructure.

A one-way (forward-only) ground-station pipeline that turns a recorded drone +
ROV bridge-inspection mission (ROS 2 rosbag2 `.mcap`) into a reviewed inspection
report, shown in the **Hyperion** web dashboard. It covers two directions from
the track brief: defect detection and segmentation on inspection imagery
(drone camera + ROV sonar), and rule-based condition rating with a recommended
action per defect.

Design rule: **code enforces, models explain.** Decoding, indexing, retrace,
geometry, correlation, severity, evaluation, hardware status and the sign-off
gate are deterministic Python. The two LLM agents (report + dashboard) only
narrate and lay out the structured result, and every report claim is validated
against the detection data.

Docs: [`PIPELINE.md`](PIPELINE.md) (the flow), [`ARCHITECTURE.md`](ARCHITECTURE.md)
(design, implemented vs. planned), [`DATA_INPUTS.md`](DATA_INPUTS.md) (data contract).

## Flow

```
1-2 decode + index    .mcap -> captures -> time/pose index                (rosbag.py, indexing.py, extract_rov.py)
3   detect            drone camera YOLOv8-seg; ROV sonar YOLO11s          (detection.py, rov.py)
4   retrace           ROV camera frame co-located with each sonar hit,    (indexing.py, rov.py)
                      clarity-gated on measured frame contrast
5   geometry          surface roughness from the LiDAR / sonar world maps (lidar.py)
6   location          element assignment from defect world positions      (correlation.py)
7   correlate+severity merge duplicates, change vs prior cycle, rule      (correlation.py, severity.py)
                      severity + rule trail + recommended action
8   report agent      Claude drafts; code drops uncited/unknown-ID lines  (agents.py)
9   dashboard agent   Claude lays out the metrics; values never change    (agents.py)
    sign-off          draft -> under_review -> approved / rejected        (review.py, Hyperion)
    evaluate          precision / recall vs defects_gt.json               (evaluation.py, eval_bag.py)
```

No step signals a vehicle. "Trigger on detection" is an internal retrace into
already-recorded data, keyed by timestamp + pose.

## Models

| Stage | Model | Notes |
|---|---|---|
| Drone camera | **Kapuni Model A** ([`iloveass/kompon-damage-severity-detection`](https://huggingface.co/iloveass/kompon-damage-severity-detection), YOLOv8s-seg) | crack / spalling / rebar corrosion / severe distress / microcrack, mapped to the pipeline's classes in `detection.CAMERA_CLASS_MAP`. Auto-downloaded into `weights/` |
| ROV sonar | **Sonnet Model C** ([SnippyCodes/Sonnet-Model-](https://github.com/SnippyCodes/Sonnet-Model-), YOLO11s) | Trained on side-scan seabed hazards, **not** bridge defects; every box is reported as a generic `sonar_anomaly`. No licence is stated, so the weights are downloaded at run time and never committed |
| Geometry | none (PCA plane fit, numpy) | local surface roughness, not cavity depth |
| Report + dashboard agents | Claude API (`claude-sonnet-5-5`) | fallback: local OpenAI-compatible model, then deterministic text. Opt-in: Kiro CLI (below) |

## Data

Two simulated missions on the Craigie Drawbridge, Boston (Isaac Sim + ArduPilot SITL):

| Mission | Platforms | Ground truth | Full bag |
|---|---|---|---|
| `run_20261024_123507` | drone (RGB, LiDAR, GNSS) | 9 above-water defects | 1.5 GB `.mcap` |
| `run_20261024_134808` | drone + BlueROV2 (multibeam sonar, camera, DVL) cooperating | 9 above + 4 underwater defects | 2.2 GB `.mcap` |

The full bags are too large for GitHub and are not in the repo.
**`sample_data/`** holds both missions trimmed to about 40 MB each (built with
`make_sample.py`): inspection frames + telemetry, ground truth, the ROV extract,
and the LiDAR / sonar maps kept at full resolution within 1.5 m of every defect
and frame aim point. With no `.mcap` present, `run_bag.py` decodes from the
frame sidecars instead, and a GT run on the drone+ROV sample gives detections
identical to the full bag. The sonar model needs the `.mcap`; on the sample it
uses the cached results in `rov/sonar_detections_unwarp.csv` if a finished run
was cached.

## Run

Mock mode needs only the standard library:

```bash
python run.py                     # fixtures/demo_mission.json
python -m unittest discover -s tests
```

Real missions (`pip install -r requirements.txt`):

```bash
# from the committed sample (no .mcap needed)
python run_bag.py --bag sample_data/run_20261024_134808 --detector gt --out out/sample.json

# full drone+ROV bag
python extract_rov.py --bag <BAG_DIR>             # once per bag: ROV streams -> <bag>/rov/
python run_bag.py --bag <BAG_DIR> --detector model --conf 0.15 \
    --rov-detector model --sonar-stride 5 --out out/coop.json
python eval_bag.py --result out/coop.json --bag <BAG_DIR> --out out/coop_eval.json
```

- `--detector model|gt`: drone detections from the camera model, or from `defects_gt.json` (labelled ground truth).
- `--rov-detector model|gt|none`: ROV detections from the sonar model (default with `--detector model`), underwater ground truth, or none.
- `--sonar-stride N`: run the sonar model on every Nth ping. It takes about 1 s per ping on CPU, so stride 5 takes about 5-7 min. Results are cached in `<bag>/rov/sonar_detections_<mode>.csv`; a cache from an unfinished run is ignored. `--rerun-sonar` forces a fresh run.
- GT defects below the water line (z < 0) belong to the ROV.

> Windows: the entry points set `KMP_DUPLICATE_LIB_OK` / `OMP_NUM_THREADS=1` to avoid a torch/numpy OpenMP crash.

### Hyperion dashboard

```bash
python annotate_frames.py --bag <BAG_DIR> --conf 0.15                     # optional: masks/boxes + H.264 feed
python export_web.py --bag <BAG_DIR> --result out/coop.json --eval out/coop_eval.json
python serve.py --bag <BAG_DIR> --port 8099                               # http://127.0.0.1:8099
```

**Bridge Defects tab**
- metric cards with a one-line meaning each
- 3D view: LiDAR map above water plus sonar map below, with defect markers by severity
- inspection-frame feed
- per-element condition cards with the recommended action
- the AI report, with per-claim detection-ID citations
- detections table
- accuracy panel (from `--eval`)
- detail drawer (click a row, card item or 3D marker): why the severity was set (rule trail from `severity.py`), the recommended action, what the class means, what the measurement means, how it was captured, reliability, and location

**Sign-off gate:** the report has a review state, draft → under review → approved / rejected.
- Approve and reject need a named reviewer.
- The server enforces the allowed transitions (`pipeline/review.py`).
- Each approval is pinned to a hash of the exact report it approved. A re-run that changes the report resets the state to draft.
- Records are stored in `out/reviews/`.

**Platform Health tab**
- **Real:** drone mission and data quality from telemetry (utilization, RTK/HDOP, LiDAR returns), and ROV navigation and sensing from the bag (DVL lock, drift vs truth, depth, camera visibility, coop dives).
- **Simulated, and labelled as such:** hardware health for both vehicles (battery and cells, motor/thruster temperatures, companion computer, data links, leak sensors), with OK/WARN/FAULT set by thresholds in `pipeline/hardware.py`. The bags record no hardware telemetry.

## Results

| Run | Scope | Found | Precision | Recall |
|---|---|---|---|---|
| Camera model, `run_20261024_123507` | 9 drone GT | 4 of 9 | 0.40 | 0.44 |
| Camera model, `run_20261024_134808` | 9 drone GT | 3 of 9 | 0.38 | 0.33 |
| Sonar model (stride 5, unwarped fan), `run_20261024_134808` | 4 ROV GT | 1 of 4 (D13) | 0.07 | 0.25 |
| GT self-check, both missions | all | all | 1.00 | 1.00 |

Matching is one-to-one, closest first, within 8 m and on the same element.
Prediction positions are frame-level aim points, not pixel-level, so a tighter
radius would score the projection rather than the detector.

What this shows: the pipeline runs end to end on both missions; the detectors
are the weak link. The camera model was trained on real photos and mostly
reports low-confidence cracks on the simulated walls. The sonar model is a
generic seabed-hazard detector and barely beats chance on a forward-looking
multibeam fan. Bridge-specific training data for both is the next step.

## Status

| Piece | State |
|---|---|
| `.mcap` decode, time/pose index, frame-sidecar fallback | real |
| Drone camera detector | real model inference |
| ROV streams (sonar, camera, DVL nav, status, coop tasks) | real, from the drone+ROV bag |
| ROV sonar detector | real inference, generic model (`sonar_anomaly`) |
| ROV camera confirm | real, clarity-gated (RMS contrast >= 0.03) |
| Geometry | real surface roughness from the LiDAR / sonar maps; no area, no cavity depth |
| Correlation, change tagging vs prior cycle, rule severity, recommended action | real (change tagging needs a prior-cycle file; only the mock fixture has one) |
| Report / dashboard agents | Claude by default; deterministic text with no key |
| Sign-off gate | real, server-enforced |
| Hardware health | **simulated**, labelled |
| Evaluation vs ground truth | real |
| Planned, not built | see the table in [`ARCHITECTURE.md`](ARCHITECTURE.md#implemented-vs-planned) |

## Agents / secrets

Configure in `.env` (copy `.env.example`; `.env` is gitignored):
- `ANTHROPIC_API_KEY`, `CLAUDE_MODEL`
- `AGENT_BACKEND`: `claude`, `local` or `kiro`

Fallback order: Kiro CLI (if selected and installed), then the Claude API, then the local model, then deterministic text. The pipeline always produces a report.

**Optional Kiro CLI backend** (`AGENT_BACKEND=kiro`): the agents call
`kiro-cli chat --no-interactive` with the prompt on stdin and no trusted tools,
using Kiro subscription credits.
- **Auth:** `KIRO_API_KEY`, or a prior `kiro-cli login`.
- **Status:** a demo option, not the default. Kiro's terms list software-development automation as the permitted automated use.

## How Kiro was used

See `KIRO.md` (added once the spec-driven work is complete). The `.kiro/`
directory holds the project's hooks and MCP settings.

## Layout

```
run.py              mock / generic live entry point
run_bag.py          run the flow on a real rosbag2 mission (or a sample_data/ mission)
extract_rov.py      ROV streams of a drone+ROV .mcap -> <bag>/rov/ CSVs + stills
eval_bag.py         score a result against defects_gt.json
make_sample.py      build a git-sized mission sample under sample_data/
annotate_frames.py  draw masks/boxes on frames + encode the H.264 feed
export_web.py       build web/data/ for Hyperion
serve.py            stdlib HTTP server for Hyperion (+ /api/review)
pipeline/
  schema.py         normalized detection record
  config.py         .env + config.json loading
  indexing.py       time/pose index + retrace
  rosbag.py         rosbag2 decode, frame-sidecar fallback, GT detections, water line
  detection.py      camera + sonar detector slots
  rov.py            ROV captures, sonar model run/cache/clustering, clarity gate, ROV health
  weights.py        download camera + sonar weights
  lidar.py          surface roughness from LiDAR / sonar maps
  correlation.py    merge, element assignment, change tagging
  severity.py       rule severity, rule trail, recommended action
  evaluation.py     GT matching + precision / recall
  health.py         platform health (real) + hardware block
  hardware.py       simulated hardware telemetry + status thresholds
  review.py         sign-off state machine
  agents.py         report + dashboard agents (Claude / local / Kiro CLI), claim validation
  orchestrator.py   mock-mode flow
scripts/            guard_env_commit.py (used by the .env commit hook)
fixtures/           offline demo mission (mock mode)
sample_data/        both missions, trimmed
web/                Hyperion UI; web/data/ is generated and gitignored
tests/              unittest suites (stdlib only)
.kiro/              hooks + MCP settings
```
