# Facts for the 3-minute video (every number here is real; nothing else may be used)

## The problem (FHWA National Bridge Inventory 2022, Massachusetts, our own analysis of data/nbi/MA22.txt)
- 5,252 bridges in Massachusetts. 444 rated Poor. Poor bridges carry about 10.1 million vehicle crossings per day.
- 2,600 cross water. 778 need underwater inspection (done by divers today). 123 are scour critical; 29 are scour critical AND Poor.
- Scour = the river eroding the bed around a bridge's foundations; it is invisible from above the water.
- Our site: the Charles River Dam crossing by the Museum of Science, Boston. The dam road bridge (built 1906, rebuilt 2011)
  carries 61,200 vehicles/day; substructure rated 4 of 9 and channel/scour protection 4 of 9 (Poor). Next to it the Craigie
  drawbridge (1910 steel bascule, 19.2 m span, 43,250 vehicles/day).
- Inspection today: snooper trucks and lane closures above, divers below, done separately.

## What we built (Hackachusetts 2026, one day, team of three; everything runs in simulation)
- Isaac Sim digital twin of the canal and drawbridge: granite walls, the bascule, the gate towers, Museum of Science.
- DRONE: Holybro X500 flown by real ArduPilot (ArduCopter SITL) through a 38-item MAVLink mission in AUTO, QGroundControl.
  Sensors: Ouster OS0-128 3D lidar, gimbal RGB camera, simulated thermal camera (FLIR Boson class, labelled simulated),
  RTK GPS that degrades under the deck (12 satellites in the open, 7 under the bridge), LED floodlights.
- ROV: BlueROV2 Heavy flown by real ArduSub SITL with NO GPS. It navigates on a simulated Doppler velocity log (DVL, Water
  Linked A50 class) fed to the autopilot's EKF, like the real BlueOS setup. Measured drift: about 0.4 m after 60 m of travel.
  Sensors: multibeam imaging sonar (130 degrees, 12 m), low-light camera with turbid-water optics, DVL bottom lock.
- TOGETHER: the drone flags defects at the water line; the ROV is automatically tasked to dive and inspect the wall directly
  below. In our recorded run the drone flagged 4 (D01 spall, D03 efflorescence, D05 crack, D07 efflorescence) and the ROV
  inspected all 4.
- One 3D model of the structure: drone lidar above the water line (2.96 million points) joined to ROV sonar below it
  (1.58 million points), 5 cm resolution, same map frame. Visible in RViz2.
- 13 ground-truth defects planted: 9 above water (spalls, cracks, efflorescence, rust streaks) and 4 underwater (a scour pit
  at the abutment toe with exposed timber piles, a missing granite block, a submerged crack, undermining at a footing).
- Everything is ROS 2: 21 topics recorded in one rosbag (469 s, 35,752 messages); shared with the team for the AI.
- AI layer: a triage model (built with Kiro) scores every one of the 5,252 Massachusetts bridges for underwater inspection
  risk from condition, scour, age and traffic. Its number 1: the North Washington Street bridge over the Charles,
  scour critical, a mile from our site.
- Honest limits: simulation only; thermal is simulated from material labels; in this demo the drone's defect flags come from
  ground truth while the detection model is trained on the recorded data.
