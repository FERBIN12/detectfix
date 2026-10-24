# How we used Kiro

**Project:** Craigie Bridge Inspector: a drone above the water line and a BlueROV2 below it inspect one bridge together,
then an AI pipeline turns the recorded mission into a cited, engineer-signed inspection report.
**Track:** Robotics (Hackachusetts 2026, challenge R6: safer inspection of critical infrastructure).
**Team:** Ferbin (robots + simulation, `simulation/`), Megha (detection pipeline + dashboard), Thiago (business case).

Everything below is in this repo: the `.kiro/` folder at the root, the Kiro task prompts and their transcripts in
`kiro_logs/`, and the code Kiro wrote.

## 1. Kiro tools we used, and how they differed

| Tool | Used for | Why that tool |
|---|---|---|
| **Kiro IDE** | The detection pipeline, the Hyperion/inspector dashboard, the specs and steering in `.kiro/` (Megha) | Interactive design: write a spec, review Kiro's design doc, then let it execute tasks one by one |
| **Kiro CLI** (`kiro-cli chat --no-interactive --trust-all-tools`) | 12 scoped, headless jobs while the simulation was running (Ferbin): see `kiro_logs/` | Long, verifiable jobs we could launch from a script and check afterwards: "build X, run its tests, write a summary" |
| **Kiro CLI as a runtime component** | `dashboard_v3/scripts/kiro_report.py` and the `kiro` backend in `pipeline/agents.py` | After every mission Kiro itself writes the inspection report from a facts JSON (see `dashboard_v3/kiro_report_example.md`) |

The CLI and the IDE complemented each other: the IDE was where we thought (specs, design), the CLI was a worker we
could hand bounded tasks to without stopping the robots.

## 2. Spec-driven development

- `.kiro/specs/bridge-world/requirements.md`: the digital twin. 8 numbered requirements (real geometry from the
  federal bridge record, defects each with a ground-truth record, standoff distances, the 6 GB VRAM budget) and an
  explicit **out-of-scope** list ("ROV, sonar, scour come later").
- `.kiro/specs/risk-based-maintenance-prioritization/`: requirements **and design** for ranking bridge elements into a
  work queue, with the rule that only code computes scores.

Compared with vibe coding, the spec made Kiro predictable. Every CLI task cited requirement numbers and a definition of
done. Example (`kiro_logs/transcripts/03_ros2_bridge_thermal.log`): Kiro built the ROS 2 publishers and a simulated
thermal camera, came back with 12 passing tests, and **reported** an unrelated failing route test as out of scope
instead of "fixing" it. That behaviour came straight from the spec's scope lines.

## 3. Steering docs (`.kiro/steering/`)

| File | Inclusion | What it changed |
|---|---|---|
| `code-enforces-models-explain.md` | always | Detection, severity and scoring stay deterministic; LLMs only narrate and must cite a detection id. Every report Kiro writes follows it. |
| `product.md`, `tech.md`, `structure.md` | always | Product scope, stack (stdlib-only mock mode), repo layout: fewer wrong-file edits |
| `sim-project.md` | always | Every number shown to a user must come from the rosbag, the ground truth or the FHWA NBI file; estimates are labelled |
| `sim-safety.md` | always | Never stop a running simulation, test only on port 8100, stay in scope, never commit as someone else |
| `sim-ros.md` | fileMatch `simulation/sim/**/*.py` | ROS 2 frames, QoS, the Jazzy "marker fields must be floats" crash, the test command |

The biggest difference came from the "every number needs a source" rule: the dashboard and the Kiro-written report
only show values computed from the recorded mission, and say so.

## 4. Agent hooks (`.kiro/hooks/`)

| Hook | Trigger | Workflow it automated |
|---|---|---|
| `block-env-commit` | PreToolUse (shell) | Blocks any git add/commit that would stage a `.env` (API keys never reach git) |
| `tests-after-spec-task` | PostTaskExec | A spec task only counts as done if the whole suite is green |
| `tests-on-pipeline-save` | PostFileSave `pipeline/*.py` | Re-runs the suite on every pipeline save |
| `sim-run-tests-on-save` | fileEdited (ROS publisher, thermal model, route) | Runs the simulation tests and asks Kiro to fix only the edited file |
| `sim-numbers-check` | fileEdited (film script, facts) | Kiro lists every number in the narration with its source line and flags unsourced ones |

## 5. MCP

`.kiro/settings/mcp.json` configures the **GitHub MCP server** (repos, pull requests, issues toolsets) for the IDE, so
Kiro can work with this repo's PRs and issues directly. The token comes from an environment variable, never the file.

## 6. What Kiro built

| Piece | Where | Kiro tool |
|---|---|---|
| Detection pipeline: rosbag decode, YOLOv8-seg defects, sonar model, retrace into lidar, rule-based severity, cited report, PE sign-off gate | `pipeline/`, `run_bag.py`, `eval_bag.py` | IDE |
| Hyperion dashboard (two-tab UI, health tab) | `web/`, `export_web.py`, `serve.py` | IDE |
| Dashboard v3 for the two-robot mission: combined lidar + sonar 3D map, GPS tracks on the Boston map, satellites over time, 5 telemetry charts, mission stats, sensor feeds, cooperation timeline, sign-off control | `dashboard_v3/` | CLI (prompts 09-12) |
| Kiro-written inspection report at runtime (compares with the previous mission, NBI facts, costs, detections vs ground truth, limits) | `dashboard_v3/scripts/kiro_report.py` | CLI, at runtime |
| ROS 2 publishers for RViz2 + simulated thermal camera, with tests | `simulation/sim/ros_pub.py`, `simulation/sim/thermal.py`, `simulation/tests/` | CLI (prompts 03-04) |
| Launch / stop scripts and RViz config for the live stack | `simulation/scripts/`, `simulation/rviz/` | CLI (transcript 02) |
| Bridge-record analysis over the FHWA NBI 2022 Massachusetts file, with 20 tests | `simulation/ai/bridge_triage.py` | CLI (transcript 01) |
| First drafts of the demo-film narration from a facts file (the final script was rewritten by hand) | `kiro_logs/prompts/06-08` | CLI |

## 7. Evidence

- `kiro_logs/prompts/`: the exact task prompts we gave Kiro CLI (01-02 survive only as transcripts).
- `kiro_logs/transcripts/`: Kiro's own transcripts of those runs (personal emails redacted).
- `simulation/KIRO.md`: the simulation-side write-up.

## Not used (honestly)

Property-based testing and Kiro powers were not used in this build. The next step for PBT is the frame conversions in
`simulation/sim/route_dam.py` and the voxel map in `simulation/sim/ros_pub.py`.
