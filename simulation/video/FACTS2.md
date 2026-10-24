# Facts v2 for the 3-minute film. Every number must come from here. "(est.)" = the team's own estimate; say so on air.

## Pain (sources: FHWA National Bridge Inventory 2022 MA file; Boston 25 News 2025; FHWA-HIF-18-049; Kentucky TC bulletin 2022-12)
- 778 Massachusetts bridges need underwater inspection. 123 are scour critical. 1.6 million vehicles a day cross those 123.
- 444 Massachusetts bridges are rated Poor (of 5,252).
- MassDOT's dive unit has 7 full-time divers.
- About 296 underwater inspections a year in Massachusetts, about $2.7M a year of work (est., at Kentucky's ~$9.3k per inspection).
- One dive-team day costs $6.8k to $10.8k per bridge (FHWA field study, 2012 dollars).
- Scour = the river eroding the bed around foundations; invisible from above the water. (Do NOT claim it is the top cause of failures.)
- Above about 3 ft/s of current divers lose mobility. Floods are when scour matters, and nobody can dive then.
- A deep dive's ~30 minutes of bottom time checks about 120 square feet of pier (FHWA).
## The rule and the insight
- FHWA guidance (2009): sonar "could only be used to supplement bridge inspection diving operations". The 2022 NBIS still requires a qualified diver and team leader.
- A federal dam inspector this week: "I'd send an ROV down, give a visual of the entire basin... send divers in and do specific spot checks."
- So: divers stay. The robot goes first. Divers only where the map says.
## Our site
- Charles River Dam crossing by the Museum of Science, Boston: dam road bridge built 1906, 61,200 vehicles/day, substructure and channel each 4 of 9 (Poor). Craigie drawbridge 1910, 43,250/day.
## What we built in one day (simulation)
- NVIDIA Isaac Sim: a robotics simulator with real-time ray-traced rendering and physics, used to test robots in a digital twin before the real thing. We rebuilt the actual canal and drawbridge from the federal record and photos.
- Inside it run the REAL ArduPilot autopilots (software-in-the-loop): ArduCopter flies a Holybro X500 drone through a 38-item mission; ArduSub flies a BlueROV2 Heavy. Everything talks ROS 2 (21 topics recorded).
- Drone: Ouster 3D lidar, gimbal camera, simulated thermal camera, floodlights. GPS drops from 12 satellites in the open to 7 under the deck.
- ROV: no GPS underwater; navigates on a Doppler velocity log (DVL) fed into the autopilot; drift about 0.4 m after 60 m. Multibeam sonar (130 degrees, 12 m) and a low-light camera.
- Together: the drone flags defects at the water line; the ROV is tasked automatically to dive below each one. 4 flagged, 4 inspected in our recorded run.
- One 3D model: 2.96 million lidar points above the water + 1.58 million sonar points below.
- 13 planted ground-truth defects: 9 above water, 4 underwater (scour pit with exposed timber piles, missing granite block, submerged crack, undermining).
## The AI, built with Kiro (Kiro = AWS's agentic AI coding IDE)
- Megha built the whole detection pipeline and inspector dashboard with Kiro: it decodes the rosbag, runs a YOLOv8 segmentation defect model on the drone imagery, retraces into the recorded lidar for each hit, measures surface roughness, assigns severity by rules, and drafts the report with an AI agent that must cite a detection id for every claim. A professional engineer signs off. "Code enforces, models explain."
- Kiro also built the statewide triage: it scores all 5,252 Massachusetts bridges for underwater-inspection risk; number 1 is the North Washington Street bridge over the Charles, scour critical, a mile from our site.
- Kiro also wrote parts of our robot software: the ROS 2 publishers, the simulated thermal camera, launch scripts and tests.
## Money (est. unless sourced)
- Divers only: $6.8k-$10.8k per bridge. Robot first, then a targeted dive: $5.3k-$7.6k (our ~$3-4k survey plus a third of a dive day) = about 20-30% less per bridge (est.; the pilot measures it).
- Our kit at list price: $16,448 (BlueROV2 $4,900, Heavy kit $1,008, Ping360 sonar $2,650, DVL A50 $7,890). FHWA-tested sonar systems cost $35k-385k for the sonar alone.
- Business: high-water scour checks (retainer + per event), robot-first survey ~$3-4k per bridge, software ranking + records ~$25-50k per agency per year (est.).
## Why now
- SNBI: by 15 March 2028 every state must re-code its bridge data, including scour, in the new federal format.
- Pilot money exists: FHWA STIC up to $125k a year; AID Demonstration up to $1M. New Hampshire DOT is already running a $183k ROV study (2023 to Sep 2026).
- Next: a 10-bridge pilot, every robot map checked against a diver.
## Team: Ferbin (robots + simulation), Megha (AI pipeline + dashboard, with Kiro), Thiago (business case). Hackachusetts 2026, robotics track.
