---
inclusion: always
---
# Craigie Bridge Inspector: project steering

- Domain: robot-first underwater bridge inspection. A drone (ArduCopter SITL) above the water line and a BlueROV2 Heavy
  (ArduSub SITL, no GPS, DVL navigation) below it, in an Isaac Sim twin of the Charles River Dam crossing, Boston.
- World frame: metres, X east, Y north, Z up, water line z = 0. Canal frame helpers live in `sim/route_dam.py` (`cw`, `C`, `R`).
- Every number shown to a user (dashboard, report, film) must come from the recorded rosbag, `data/defects_gt.json`, or the
  FHWA NBI 2022 Massachusetts file. Team estimates must be labelled "our estimate".
- Honest limits are always stated: simulation only; thermal camera is simulated; waterline flags come from ground truth.
- Code enforces, models explain: detection, severity and statistics are deterministic Python; LLM agents only narrate and
  must cite a detection id for every claim. A licensed engineer signs off.
