# Project structure

```
run.py              mock / generic entry point (uses pipeline/orchestrator.py)
run_bag.py          full flow on a rosbag2 mission or a sample_data/ mission
extract_rov.py      ROV streams of a drone+ROV .mcap -> <bag>/rov/ CSVs + stills
eval_bag.py         score a result JSON against defects_gt.json
make_sample.py      build a git-sized mission sample under sample_data/
annotate_frames.py  draw masks/boxes on frames + encode the H.264 feed
export_web.py       build web/data/ for Hyperion
serve.py            stdlib HTTP server for Hyperion (+ /api/review)
pipeline/           all pipeline logic (see below)
tests/              unittest suites, stdlib only
fixtures/           offline mock mission, element map, prior-cycle detections
sample_data/        both missions trimmed to ~40 MB (frames + sidecars, GT, ROV extract, maps)
web/                Hyperion UI (index.html, app.js, style.css); web/data/ is generated
scripts/            guard_env_commit.py (used by the .env commit hook)
.kiro/              steering, hooks, MCP settings
```

Gitignored and generated, so never edit by hand or commit: `out/` (run results, `out/reviews/` sign-off records), `weights/` (downloaded models), `web/data/`, `.env`, and scratch files matching `_*.py` / `_*.txt` / `_*.log`.

## pipeline/ modules, by flow step
| Step | Module | Owns |
|---|---|---|
| contract | `schema.py` | the normalized `Detection` record, class lists, `make_detection` |
| config | `config.py` | `.env` + `config.json` loading |
| 1-2 decode/index | `rosbag.py`, `indexing.py` | `.mcap` decode, frame-sidecar fallback, GT detections, water line, time/pose index + retrace |
| 3 detect | `detection.py`, `rov.py`, `weights.py` | camera + sonar detectors, `CAMERA_CLASS_MAP`, sonar run/cache/clustering, weight download |
| 4 retrace | `indexing.py`, `rov.py` | ROV camera confirm, clarity gate (RMS contrast) |
| 5 geometry | `lidar.py` | surface roughness from LiDAR / sonar maps |
| 6-7 locate/correlate | `correlation.py` | merge, element assignment, change tagging |
| 7 severity | `severity.py` | rule severity, rule trail, recommended action |
| 8-9 agents | `agents.py` | report + dashboard agents, `_validate_report` claim checker |
| sign-off | `review.py` | review state machine, reviewer name, report-hash pinning |
| evaluate | `evaluation.py` | one-to-one GT matching (8 m, same element), precision / recall |
| health | `health.py`, `hardware.py` | real platform health; simulated hardware + OK/WARN/FAULT thresholds |
| mock flow | `orchestrator.py` | wires the steps for mock mode |

## Where changes go
- A new fact or decision goes in the deterministic module that owns it (table above), never in an agent prompt.
- A new defect class goes in `schema.py` (`DRONE_CLASSES` / `ROV_CLASSES`) plus its rules in `severity.py`. Camera model labels map through `detection.CAMERA_CLASS_MAP`.
- New CLI entry points go at the repo root, with the logic kept in `pipeline/`.
- Keep the docs in sync when behavior changes: `README.md` (run + status), `PIPELINE.md` (flow), `ARCHITECTURE.md` (design, implemented vs planned), `DATA_INPUTS.md` (data contract, mirrored by `schema.py`).
- GT defects with z < 0 (below the water line) belong to the ROV.
