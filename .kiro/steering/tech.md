# Tech stack

- **Language:** Python 3 (uses `from __future__ import annotations`, dataclasses, type hints). No build system or packaging. Scripts run directly.
- **Mock mode and tests:** standard library only. This is a hard rule.
- **Live mode** (`requirements.txt`): `ultralytics` (YOLOv8/YOLO11), `huggingface_hub` (weight download), `numpy`, `open3d` (optional, pure-Python fallback exists), `mcap` + `mcap-ros2-support` (rosbag2 decode, JSON-sidecar fallback exists), `imageio` + `imageio-ffmpeg` (H.264 feed), `python-dotenv`.
- **LLM agents:** Claude API (`claude-sonnet-5-5` by default) called with stdlib `urllib`, with no SDK. Fallback order: Kiro CLI (if `AGENT_BACKEND=kiro`) → Claude → local OpenAI-compatible model → deterministic text.
- **Web (Hyperion):** plain `web/index.html`, `app.js`, `style.css`, served by stdlib `serve.py`. No JS build step.
- **Config:** `config.json` (mode, retrace gates, paths) plus `.env` (secrets, agent backend). `pipeline/config.py` loads both.

## Conventions
- Import third-party packages inside the function that needs them, never at module top level, so mock mode still imports cleanly.
- Every threshold is a named module-level constant (UPPER_CASE) with a comment saying where the number came from (e.g. `severity.DEPTH_MM`, `severity.LOW_CONFIDENCE`).
- Build detections with `schema.make_detection(...)`, which stamps a `D-xxxxxxxxxxxx` ID and validates. Don't call `Detection(...)` directly. Classes must be in `DRONE_CLASSES` / `ROV_CLASSES`.
- In JSON the class field is named `class`. In Python it is `cls` (`Detection.to_dict()` converts it).
- Each module starts with a docstring naming its pipeline step and what it owns.
- Any stage that calls a model needs a deterministic fallback that works offline with no API key.
- Mark non-real data: `model_version` starting with `ground-truth@`, `is_simulated`, `is_filler`, `sonar_anomaly`.
- Never put secrets in code or logs. `.env` is gitignored, and a PreToolUse hook (`scripts/guard_env_commit.py`) blocks committing it.
- Windows: entry points set `KMP_DUPLICATE_LIB_OK` and `OMP_NUM_THREADS=1` to avoid a torch/numpy OpenMP crash. Keep this in new entry points that load torch.

## Testing
- `unittest` only, in `tests/` (`test_pipeline.py`, `test_evaluation.py`). Tests add the repo root to `sys.path` and use `fixtures/` and `sample_data/`.
- Changes to severity, correlation, matching, the report validator, hardware status or the review gate need a unit test. Add a property test where a universal statement holds. Any non-stdlib test dependency must be skipped when it isn't installed.
- Hooks run the full suite after saving any `pipeline/*.py` file and after each spec task.

## Common commands
```powershell
python run.py                                   # mock mode, fixtures/demo_mission.json
python -m unittest discover -s tests            # full test suite (stdlib)
pip install -r requirements.txt                 # live mode deps

# committed sample, ground-truth detector (no .mcap needed)
python run_bag.py --bag sample_data/run_20261024_134808 --detector gt --out out/sample.json

# full bag
python extract_rov.py --bag <BAG_DIR>           # once per bag: ROV streams -> <bag>/rov/
python run_bag.py --bag <BAG_DIR> --detector model --conf 0.15 --rov-detector model --sonar-stride 5 --out out/coop.json
python eval_bag.py --result out/coop.json --bag <BAG_DIR> --out out/coop_eval.json

# Hyperion
python annotate_frames.py --bag <BAG_DIR> --conf 0.15
python export_web.py --bag <BAG_DIR> --result out/coop.json --eval out/coop_eval.json
python serve.py --bag <BAG_DIR> --port 8099     # long-running: tell the user to start it themselves
```
The sonar model takes about 1 s per ping on CPU. Results are cached in `<bag>/rov/sonar_detections_<mode>.csv`, and `--rerun-sonar` forces a fresh run.
