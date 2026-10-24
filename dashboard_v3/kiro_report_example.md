# Craigie Bridge / Charles River Dam — Inspection Report

## Summary

A two-robot mission (id `run_20261024_134808`) surveyed the Craigie Drawbridge and Charles River Dam lock canal walls in Boston. A drone (ArduCopter, 418.1 s flight, 203.0 m, 37 waypoints) inspected the structure above the water line while a BlueROV2 (ArduSub, 418.1 s dive, 201.1 m, max depth 3.74 m) surveyed the submerged walls and canal bed. The drone flagged 4 waterline defects and handed each to the ROV for a targeted dive; against 13 ground-truth defects the above-water camera model confirmed only 1 (D04 rust streak), while the 4 underwater defects were covered by the ROV's multibeam sonar survey route.

## Findings by element

### Above the water line (drone)

- 9 ground-truth defects sit above the water line; the drone RGB model caught **1** (D04 rust streak, matched as `corrosion_stain` at 6.0–6.6 m) and missed the other 8 (D01, D02, D03, D05, D06, D07, D08, D09). One extra `crack` detection had no GT match within 8 m.
- Of 13 GT defects, only 3 model detections were produced (all "fair", none poor/severe) — detection runs on drone RGB only.
- **4 waterline defects were flagged and handed to the ROV** (z ≈ 0.8–1.5 m), all on the canal walls: D03 efflorescence (canal_wall_E, z 0.8 m), D01 spall (canal_wall_E, z 1.0 m), D07 efflorescence (canal_wall_W, z 1.5 m) and D05 crack (canal_wall_W, z 1.2 m). For each, the ROV was tasked to dive and inspect the submerged wall directly below.

### Below the water line (BlueROV2)

4 underwater ground-truth defects lie along the ROV's survey route and were covered by its multibeam sonar survey (not drone handoffs, and no trained sonar model was run):

- **D10 scour** — abutment_E_toe, depth 5.2 m; 2.62 m from the nearest ROV ground-truth path point.
- **D11 missing_block** — canal_wall_E_submerged, depth 2.2 m; 1.50 m from the nearest path point.
- **D12 crack** — canal_wall_E_submerged, depth 1.6 m; 1.77 m from the nearest path point.
- **D13 undermining** — canal_wall_W_footing, depth 3.85 m; 2.40 m from the nearest path point.

DVL dead-reckoning held bottom lock 100% of the dive, with a final navigation error of 0.16 m (max 0.599 m).

## Comparison with the previous mission

The previous run (`run_20261024_123507`) was drone-only — no ROV, sonar, or cooperation. New this run:

- Underwater survey with the BlueROV2 (ArduSub), absent previously.
- Multibeam sonar map of the canal bed and submerged walls (1,475 sonar frames).
- DVL dead-reckoned navigation with measured ground-truth error.
- Drone→ROV cooperation (4 waterline flags handed to the ROV to dive).

## Statewide context (FHWA NBI 2022)

The two dam-crossing structures (both carrying ST 28 CHAS R DM RD over the Charles River, scour code 8):

- **Charles River Dam 10-span bridge** (B160134ETDOTNBI, built 1906): lowest condition rating 4 (Poor, bridge condition "P"), ADT 61,200; underwater inspection required every 24 months, last 2020-05-01, due 2022-05-01 — 53 months overdue.
- **Craigie drawbridge** (B160134EMDOTNBI, steel bascule, built 1910): lowest condition rating 6 (Fair, bridge condition "F"), ADT 43,250; underwater inspection required every 48 months, last 2021-03-01, due 2025-03-01 — 19 months overdue.

Statewide (2022 file): 5,252 bridges total, 2,600 over water, 444 in Poor condition, 123 scour-critical (29 both scour-critical and Poor), and 778 requiring underwater inspection — of which 774 are overdue.

Cost: diver-only inspection runs about $6,800–$10,800 per bridge per dive-team day (FHWA 2012 dollars), versus roughly $5,300–$7,600 for robot-first screening plus one targeted dive (team estimate).

## Recommended next dive spots

Dive the submerged wall directly below each of the 4 drone-flagged waterline defects (the cooperation handoffs):

- **Below D03** (efflorescence, canal_wall_E) → dive point (-29.173, -29.533, -3.5).
- **Below D01** (spall, canal_wall_E) → dive point (-25.094, -23.844, -3.5).
- **Below D07** (efflorescence, canal_wall_W) → dive point (9.944, 2.714, -3.5).
- **Below D05** (crack, canal_wall_W) → dive point (-7.537, -21.667, -3.5).

## Limitations

- This is a simulated mission (Isaac Sim + ArduPilot SITL), not a live bridge.
- The thermal feed is simulated LWIR.
- Waterline defect flags use ground truth in this demo (no live operator).
- No trained sonar defect model is run; the 4 underwater defects are ground truth covered by the ROV's sonar survey route, not model detections.
- NBI "overdue" is measured against the 2022 file; dives newer than 2022 are not in it.
