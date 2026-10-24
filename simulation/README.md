# Craigie Bridge Inspector (Hackachusetts 2026, Robotics track, challenge R6)

Safer inspection of critical infrastructure: a drone flown by **real ArduPilot** inspects the **Craigie Drawbridge and the
Charles River Dam lock canal** (Route 28 by the Museum of Science, Boston) in Isaac Sim. In the National Bridge
Inventory the Charles River Dam Road bridge here is rated Poor (substructure 4/9, channel and scour protection 4/9) and
the Craigie bascule (19.2 m main span, 1910) Fair. Inspecting them today means
snooper trucks, lane closures and divers. The drone flies the canal walls and threads under the deck, where GPS fails.

## What runs

| Piece | What it is |
|---|---|
| Autopilot | ArduCopter SITL (real ArduPilot flight code, EKF, GPS), home = an inspection RIB in the canal |
| Mission | `sim/route_dam.py` -> 38 MAVLink mission items uploaded by `sim/mav_bridge.py`, flown in AUTO |
| GPS under the deck | `mav_bridge.py` lowers SITL's satellite count while the drone is under the bridge, so the autopilot and QGC really see degraded GNSS |
| World | `sim/charles_dam_world.py`: granite ashlar canal walls, the steel bascule (DCR green), gate towers, Lechmere viaduct, Museum of Science, traffic, path-traced stills |
| Drone | `sim/x500.py`: Holybro X500 V2 from PX4's meshes, CGO3 gimbal, Ouster OS0-128 lidar, RTK GNSS, LED floodlights + upward soffit light, nav lights, strobe |
| Ground station | QGroundControl on tcp 5762 |
| ROS 2 / RViz2 | `sim/ros_pub.py` publishes from inside Isaac; `rviz/inspection.rviz` shows it |
| Thermal | `sim/thermal.py`: **simulated** LWIR (FLIR Boson 640 class) from semantic labels + per-material temperatures, attenuation, blur and NETD noise. Isaac has no physics thermal sensor; this is an approximation and labelled as one |

### ROS 2 topics (Jazzy)

`/tf` `/tf_static` (map -> odom -> base_link -> os0_lidar / gimbal_cam), `/odom`, `/path` (flown trajectory),
`/mission` + `/mission/markers` (planned route), `/lidar/points` (OS0-128, sensor frame), `/map` (accumulated lidar,
0.10 m voxels), `/camera/rgb`, `/camera/thermal`, `/defects` (ground-truth defect markers), `/drone/status`.

Measured headless on an RTX workstation: ~16 Hz odom/lidar, RGB ~4 Hz, thermal ~3 Hz, map 1 Hz.

## Run

    scripts/run_live.sh      # SITL -> Isaac world -> mission bridge -> QGroundControl -> RViz2
    scripts/stop_live.sh     # stops exactly the PIDs run_live.sh started
    scripts/rviz.sh          # RViz2 alone

Stills: `STILL=1 HEADLESS=1 OMNI_KIT_ACCEPT_EULA=YES ~/IsaacLab/env_isaaclab/bin/python sim/charles_dam_world.py`
-> `data/stills/`. Env: `ROS=0` disables ROS publishing, `PROF=1` prints loop timings.

Tests: `source /opt/ros/jazzy/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ~/IsaacLab/env_isaaclab/bin/python -m pytest tests -q`

## Credits

- Holybro X500 V2 and CGO3 gimbal meshes: PX4-gazebo-models (BSD-3-Clause).
- Scanned textures: Poly Haven (CC0).
- Zodiac RIB model by DenisGLabrecque on Sketchfab (CC BY).
- Bridge data: FHWA National Bridge Inventory 2022 (public domain).
- Water material: our own Harbour USV project.
- Pre-built before the event (disclosed): the water material and photoreal helpers from our earlier projects, and the ROV scene in `sim/rov_arm_scene.py` (not used in the demo).

Built at Sundai.
