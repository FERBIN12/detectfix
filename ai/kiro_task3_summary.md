# Kiro Task 3 — ROS 2 visualization bridge + simulated thermal camera

Scope: new files only under `~/hackachusetts`. No existing `sim/*` file was
modified and no process was started/stopped. Everything targets
`~/IsaacLab/env_isaaclab/bin/python` (Python 3.12, numpy 2.5.3) after
`source /opt/ros/jazzy/setup.bash`, using only numpy, cv2, rclpy and standard
ROS message packages.

## What I built

### `sim/ros_pub.py` — `RosPub(node_name="inspection_drone")`
Non-blocking ROS 2 publishing facade meant to be driven from the Isaac main loop.
`rclpy.init()` only if not already initialised. All methods are cheap; the caller
pumps the executor via `spin()` → `rclpy.spin_once(node, timeout_sec=0.0)`.

- `pose(pos, quat_xyzw, vel=None)` — dynamic TF `odom→base_link`,
  `nav_msgs/Odometry` on `/odom` (frame `odom`, child `base_link`), and appends to
  `nav_msgs/Path` `/path` (frame `map`) only after ≥ 0.15 m of travel, publishing
  `/path` at ≤ 5 Hz. A static identity `map→odom` is published once at
  construction on `/tf_static` with TRANSIENT_LOCAL QoS (built directly from
  `tf2_msgs/TFMessage`, no `tf2_ros` dependency).
- `sensor_tf(child, pos, quat_xyzw, parent="base_link")` — dynamic sensor TF.
- `cloud(P, frame="os0_lidar", topic="/lidar/points")` — `PointCloud2` with
  x/y/z float32 fields built straight from the numpy byte buffer (`tobytes`), no
  per-point Python loop.
- `map(Pw)` — voxel-downsamples world points to 0.10 m via integer voxel keys +
  `np.unique`, caps at 400k points, publishes `PointCloud2` on `/map` (frame
  `map`) with an `intensity` field equal to `z` (height colouring), latched and
  rate-limited to 1 Hz.
- `image(topic, img, frame, encoding="rgb8")` — `sensor_msgs/Image` from HxWx3
  uint8, or HxW float32 with `"32FC1"`; rate-limited per topic to 10 Hz.
- `mission(waypoints)` — `/mission` (`Path`, frame `map`) + `/mission/markers`
  (`MarkerArray`: spheres per waypoint, a LINE_STRIP through them, TEXT labels at
  the first waypoint of each new element), TRANSIENT_LOCAL.
- `defects(list_of_dicts)` — `/defects` (`MarkerArray`, frame `map`,
  TRANSIENT_LOCAL): a cube coloured red/orange/yellow by severity plus a
  `"kind (severity)"` text label. Accepts both `kind`/`type`, `position`/`pos`,
  and int or string severities (adapted to the real `data/defects_gt.json` format,
  which uses `type` and integer severity 1–3).
- `status(text)` — `std_msgs/String` on `/drone/status`.
- `spin()`, `close()`.

Frames: `map, odom, base_link, os0_lidar, gimbal_cam, thermal_cam`. All stamps
come from the node clock.

### `sim/thermal.py` — simulated LWIR camera (FLIR Boson 640 class, 640×512)
Honest approximation (stated in the module docstring): Isaac has no physics
thermal sensor, so this paints per-class base temperatures onto a semantic label
image and adds plausible structure — it is not radiometric.

- `synth(labels, id2class, depth, t, ambient_c=12.0, rng=None) -> (temp_c float32 HxW, ironbow uint8 HxWx3)`
  — labels/depth resized to 640×512 (labels with INTER_NEAREST) *before* colouring;
  per-class offsets (sky −25, water −3, granite 0, granite_wet −2.5, concrete +1,
  asphalt +6, steel +2, steel_rust +2.5, brick +1.5, vegetation −1, vehicle +15,
  boat +3, drone +8, defect_moisture −4, defect_delamination +3.5, defect_crack
  −1.5, default 0); unknown ids → `sky`. Adds stable low-frequency spatial
  variation (±0.6 °C, seeded per class set), atmospheric attenuation toward
  ambient (`tau = exp(-depth/400)`), a σ≈1 px Gaussian optical blur, and ~0.05 °C
  temporal NETD noise. Auto-ranges per frame on the 2nd/98th percentile with a
  minimum 6 °C span.
- `ironbow(temp_c, lo, hi)` — black→blue→purple→red→orange→yellow→white palette
  via monotone control-point interpolation, returns uint8 HxWx3 RGB.

### `rviz/inspection.rviz`
RViz2 config, fixed frame `map`. Displays: Grid; TF (axes + small markers);
Odometry `/odom` (red arrow, keep 1); Path `/path` (green); Path `/mission`
(white, billboard line); MarkerArray `/mission/markers` and `/defects`;
PointCloud2 `/lidar/points` (intensity colour, 0.03 m points); PointCloud2 `/map`
(AxisColor Z, 0.05 m, decay 0); Image `/camera/rgb` and `/camera/thermal`. Orbit
view, focal point at origin, distance 60 m, pitch 0.5. Validated as well-formed
YAML (top keys `Panels`, `Visualization Manager`, `Window Geometry`).

### `scripts/rviz.sh`
Resolves the repo root relative to the script, sources
`/opt/ros/jazzy/setup.bash`, and `exec rviz2 -d <repo>/rviz/inspection.rviz`.
`chmod +x`, `bash -n` clean.

### Tests — `tests/test_ros_pub.py`, `tests/test_thermal.py`
Modules imported by path (like `test_route.py`). Coverage: voxel-downsample
correctness (collapse within a voxel / keep separate voxels / empty), PointCloud2
byte-layout round-trip via `sensor_msgs_py.point_cloud2.read_points_numpy` (xyz
and xyzi), `/map` intensity == z, `/path` ≥ 0.15 m decimation logic, defect cube
colour-by-severity, thermal output shapes/dtypes, defects colder than granite in
`temp_c`, ironbow monotone brightness, and ironbow black/white endpoints.

## How to run the tests
The ROS 2 Jazzy `launch_testing*` pytest plugins fail to load in this env
(missing `lark`, plus a `pytest_launch_collect_makemodule` hook incompatibility),
which crashes collection. The clean, non-invasive workaround is to disable pytest
plugin autoloading — the new tests only need core pytest:

```
source /opt/ros/jazzy/setup.bash
cd ~/hackachusetts
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ~/IsaacLab/env_isaaclab/bin/python -m pytest \
    tests/test_ros_pub.py tests/test_thermal.py -q
```

## Exact test result

```
............                                                             [100%]
12 passed in 1.49s
```

All 12 tests in the two new files pass.

Note on `pytest tests` (whole directory): `tests/test_route.py` (a pre-existing
test for `sim/route.py`, outside this task's scope) reports one geometry failure
— `pier_3 waypoint [14.3 -7.85 1.8] only 2.79 m from pier (need >= 2.9 m)`. That
is unrelated to the files written here; `sim/route.py` and `tests/test_route.py`
were left untouched per the "do not modify existing files" constraint. The other
17 tests in the directory (including all of mine) pass.
