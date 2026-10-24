> **Trimmed sample**: no `.mcap` and no `.pcd`/`.ply` maps. Kept: inspection frames, telemetry, `defects_gt.json` and `map/lidar_map_world.npy`. The original bag README follows.

# Craigie Bridge inspection flight, 2026-10-24 12:35 (Isaac Sim + ArduCopter SITL)

One full mission: launch from the RIB, east canal wall (two heights), under the drawbridge girders, west wall, RTL.
Recording runs 418 s and continues ~1-2 min past landing (drone parked on the pad).

## Rosbag (MCAP, zstd): `run_20261024_123507_0.mcap` + `metadata.yaml`
    source /opt/ros/jazzy/setup.bash
    ros2 bag info .            ros2 bag play .            (or open the .mcap in Foxglove)

| Topic | Type | Count | Notes |
|---|---|---|---|
| /camera/rgb | sensor_msgs/Image rgb8 960x540 | 1056 | gimbal camera (CGO3), frame gimbal_cam |
| /camera/thermal | sensor_msgs/Image rgb8 640x512 | 882 | SIMULATED LWIR, ironbow, auto-ranged per frame |
| /lidar/points | PointCloud2 xyz | 4288 | Ouster OS0-128, sensor frame os0_lidar, water returns removed |
| /odom | nav_msgs/Odometry | 4289 | ArduPilot EKF pose, frame odom |
| /tf, /tf_static | TF | | map -> odom -> base_link -> os0_lidar / gimbal_cam |
| /path | nav_msgs/Path | 466 | flown trajectory |
| /mission, /mission/markers | Path / MarkerArray | 1 | planned route |
| /defects | MarkerArray | 1 | ground-truth defects (also defects_gt.json) |
| /drone/status | String | 285 | GNSS sats/HDOP/fix, ArduCopter mode, mission item |

915 best-effort messages were dropped during recording (mostly lidar at 15 Hz). /map is not in the bag; it is under map/.

## `map/`: accumulated 3D lidar map, world frame (metres, Z up), 1,114,883 points
- `lidar_map_world.pcd` (binary xyz), `lidar_map_world_5cm.pcd` (841k, 5 cm voxels), `lidar_map_world.ply` (coloured by height), `lidar_map_world.npy`

## `inspection_frames/`: one RGB still per mission waypoint (`NNNN.png`) + `NNNN.json` (element, drone pos, target, standoff, GNSS), `telemetry.csv`
## `defects_gt.json`: ground-truth defects (id, type, severity 1-3, element, position, normal), same world frame
