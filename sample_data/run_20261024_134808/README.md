> **Trimmed sample** (built by `make_sample.py`): no `.mcap`, `bag_to_maps.py` or `.pcd`/`.ply` maps. Kept: inspection frames, telemetry, `defects_gt.json`, the ROV extract (`rov/`) and the `.npy` maps at full resolution near defects. The original bag README follows.

# Craigie Bridge inspection: drone + BlueROV2 working together (Isaac Sim + ArduPilot SITL), 2026-10-24 13:48

Above water: a Holybro X500 (ArduCopter SITL) flies the canal walls and threads under the Craigie drawbridge.
Below water: a BlueROV2 Heavy (ArduSub SITL, **no GPS**, navigating on a simulated Water Linked A50-class DVL) surveys the
submerged walls, the scour pit at the east abutment toe and the footings. **Cooperation:** when the drone passes a defect
at the water line, the ROV is tasked to dive and inspect the wall directly below it (see coop_log.txt: D03, D01, D07, D05).
Recording: 469 s, stopped by hand after the drone landed (ROV survey still running).

## `run_20261024_134808_0.mcap` + metadata.yaml (ROS 2 Jazzy, MCAP/zstd)
    source /opt/ros/jazzy/setup.bash && ros2 bag play .      (or open the .mcap in Foxglove)

Drone: /camera/rgb (960x540), /camera/thermal (SIMULATED LWIR, 640x512 ironbow), /lidar/points (Ouster OS0-128, os0_lidar
frame), /odom, /path, /mission, /mission/markers, /drone/status
ROV: /rov/camera (640x360, turbid-water optics model), /rov/sonar/image (multibeam fan, 130 deg, 12 m), /rov/sonar/points
(sonar returns, map frame), /rov/odom + /rov/path (ArduSub EKF on DVL = what the ROV believes), /rov/truth/odom +
/rov/truth/path (ground truth: compare to see DVL drift), /rov/dvl/beams, /rov/status
Shared: /tf, /tf_static (map -> odom -> base_link -> sensors; map -> rov/odom -> rov/base_link), /defects (ground truth),
/coop/tasks (drone-flagged defect -> ROV dive point)
994 best-effort messages dropped while recording (mostly lidar/sonar).

## `map/`: 3D maps, world frame (metres, X east, Y north, Z up, water line z = 0), 5 cm voxels
- drone_lidar_world (2.96 M pts, above water), rov_sonar_world (1.58 M pts, below water), combined_world (4.54 M): .pcd, .ply (coloured), .npy
- map_check_crosssection_topview.jpg: cross-section across the canal: lidar above the water line, sonar below it
- rebuild from the bag: `python3 bag_to_maps.py .`

## `inspection_frames/`: drone RGB still per waypoint + JSON (element, pose, standoff, GNSS), telemetry.csv
## `defects_gt.json`: 13 ground-truth defects (9 above water, 4 underwater: scour, missing block, crack, undermining)
