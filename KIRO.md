# How we used Kiro

Craigie Bridge Inspector was built in one day at Hackachusetts 2026 (Robotics track). Kiro did the AI side end to end
and a scoped share of the robot software. This is what we used, and how.

## Tools: Kiro IDE and Kiro CLI

- **Kiro IDE** (signed in with the event credits) for the spec and the AI/dashboard work: Megha built the detection
  pipeline and inspector dashboard in the IDE ([detectfix](https://github.com/mochi-bunny/detectfix)).
- **Kiro CLI** (`kiro-cli chat --no-interactive --trust-all-tools`) as a headless worker we could hand scoped tasks to
  from scripts while the simulation was running. The CLI was the better fit for long, verifiable jobs (build module X,
  run its tests, write a summary); the IDE was better for interactive design.
- **Kiro as a runtime component, not only a dev tool:** `scripts/kiro_report.py` (detectfix) collects a facts JSON from
  the recorded mission and calls `kiro-cli` to write the inspection report that the dashboard shows, tagged "generated
  by Kiro" with a timestamp.

## Spec-driven development

`.kiro/specs/bridge-world/requirements.md` came first: 8 numbered requirements for the digital twin (real geometry,
defects with a ground-truth record each, standoff distances, the 6 GB VRAM budget) and an explicit *out of scope* list
("ROV, sonar, scour come later"). Each Kiro task then referenced requirement numbers and had a definition of done
(files, tests, a summary file). Compared with vibe coding, the difference was predictability: Kiro's task 3 (ROS 2
publishers + a simulated thermal camera) came back with 12 passing tests and, instead of "fixing" an unrelated failing
route test, it reported it as out of scope. The spec's out-of-scope list is what made that happen.

## Steering docs (`.kiro/steering/`)

- `project.md` (always): domain, world frame, and the rule that every number must come from the bag, the ground truth
  or the FHWA NBI file, with estimates labelled. This stopped invented statistics in the report and dashboard.
- `safety.md` (always): never touch processes you did not start, test only on port 8100, stay in scope, never commit as
  someone else. These rules came from real incidents during the day (an agent committing under a teammate's name).
- `ros.md` (fileMatch `sim/**/*.py`): frame names, QoS choices, the Jazzy "marker fields must be floats" crash, and the
  test command, so ROS work stays consistent with the running stack.

## Agent hooks (`.kiro/hooks/`)

- `run-tests-on-save`: saving a ROS publisher, the thermal model, the route or the triage code runs the test suite and
  asks Kiro to explain and propose a fix limited to that file.
- `numbers-check`: editing the narration or facts files makes Kiro list every number with its source line and flag any
  unsourced number or unlabelled estimate. The video's numbers went through this check.

## What Kiro built

| Piece | Where |
|---|---|
| Detection pipeline: rosbag decode, YOLOv8-seg defects, retrace into lidar, rule-based severity, cited report, PE sign-off | detectfix `pipeline/` |
| Inspector dashboard: 3D lidar + sonar map, GPS tracks, telemetry, stats, sensor feeds, cooperation timeline | detectfix `web/`, `export_web.py` |
| Kiro-written inspection report (runtime) | detectfix `scripts/kiro_report.py`, `web/data/kiro_report.md` |
| ROS 2 publishers for RViz + simulated thermal camera, with tests | `sim/ros_pub.py`, `sim/thermal.py`, `tests/` |
| Launch/stop scripts, RViz config | `scripts/run_live.sh`, `scripts/stop_live.sh`, `rviz/` |
| Bridge-data triage over the FHWA NBI 2022 Massachusetts file, with 20 tests | `ai/bridge_triage.py` |

## Not used

We did not use MCP servers, property-based tests or Kiro powers in this build; with more time, PBT is the next step for
the frame conversions in `sim/route_dam.py` and the voxel map in `sim/ros_pub.py`.
