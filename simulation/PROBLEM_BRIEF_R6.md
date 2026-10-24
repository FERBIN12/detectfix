# R6 "Safer inspection of critical infrastructure" → the underwater half: bridge scour

## The challenge (hackachusetts2026.vercel.app/#/robotics, R6)
"The drone already flew. The budget still has to choose." MassDOT Aeronautics flies drones over bridges; the bottleneck
is using the data and choosing where limited budgets go. FHWA's new SNBI format: transitioned data accepted only in the
2026 and 2027 submittals; a complete SNBI dataset is due for all reportable bridges by **15 March 2028**.
Directions: defect detection on imagery, **risk-based prioritization** (condition, age, traffic, location), **NBI → SNBI
harmonization**, **simulated inspection routes with standoff distance and coverage**. No hardware required.
Their dataset: National Bridge Inventory, Massachusetts (MA22.txt). Also dacl10k defect images.

## Our angle: drones can't see below the waterline
Scour of bridge foundations is the **most common cause of bridge failures** (LTRC report prepared with FHWA). Scour happens
underwater, so a drone cannot inspect it. Today it is done by divers on a fixed schedule.

## Numbers from THEIR dataset (MA22.txt, NBI 2022, computed today)
- 5,252 Massachusetts bridges; **2,600 cross water**.
- **778 bridges need underwater (diver) inspections** (item 92B), most every 36 months (578), some every 12 (29).
- **123 bridges are rated scour critical** (item 113 codes 0–3); **29 of them are also in Poor condition**.
- **~1.59 million vehicles a day** cross those scour-critical bridges; their median year built is **1924**.
- Busiest: a 1959 bridge over the **Merrimack River** carrying ~130,000 vehicles a day, scour critical and Poor.
  Charles River bridges from 1900, scour critical and Poor.

## What we build (uses our stack + their data)
1. **Prioritize (their data):** a risk score per bridge from items 113 (scour), 60/61 (substructure, channel), 27 (age),
   29 (traffic), 92B/93B (underwater interval + last date) → "which piers to dive or robot first" with the MA map. Map the
   scour fields to their SNBI equivalents so the score survives the 2026–2028 schema change.
2. **Inspect (Isaac):** the ROV flies a planned route around a pier at a fixed standoff, holding station in river current,
   multibeam sonar maps the riverbed → **scour hole depth map** around the pier (our sonar mosaic code, pointed at the bed).
3. **Decide:** a code rule compares measured scour depth with the foundation (critical → "Needs you" card for the
   bridge engineer; else log and re-inspect). Output: an inspection record ready for SNBI.

## Pitch (one line)
"Massachusetts flies drones over its bridges, but the #1 way bridges fail is scour under the water. 778 bridges here need
divers, and 1.6 million cars a day cross the 123 that are scour critical. We send a robot instead and tell MassDOT which
ones to fix first."

## Careful
- MA22 is the 2022 file the organizers gave us; say "NBI 2022".
- Say "scour critical" as the NBI defines it (item 113 codes 0–3), not "about to collapse".
- The "most common cause" line: cite LTRC/FHWA, not Wikipedia.
