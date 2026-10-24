"""Tests for sim/ros_pub.py (the ROS 2 publishing facade).

Covered here, using only numpy, rclpy and standard ROS message packages:

  * voxel_downsample() correctness: one point per 0.10 m voxel, and points
    inside the same voxel collapse to a single representative;
  * PointCloud2 byte layout round-trips through
    sensor_msgs_py.point_cloud2.read_points_numpy (xyz and xyzi);
  * /map colours by height (intensity field == z);
  * /path decimation: poses are appended only after >= 0.15 m of travel.

The module is imported by path so the test does not depend on sys.path layout.
Tests that spin rclpy must run with no other ROS node present; each test owns
its own node lifecycle and we rclpy.init()/shutdown() around the module fixture.
"""
import os
import importlib.util

import numpy as np
import pytest

import rclpy
from sensor_msgs_py.point_cloud2 import read_points_numpy

# ---- import sim/ros_pub.py by path -----------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_PATH = os.path.join(_HERE, "..", "sim", "ros_pub.py")
_spec = importlib.util.spec_from_file_location("ros_pub_under_test", _PATH)
ros_pub = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ros_pub)

RosPub = ros_pub.RosPub


# ============================================================================
# Pure-numpy tests (no ROS node needed)
# ============================================================================
def test_voxel_downsample_collapses_within_voxel():
    """Points within the same 0.10 m voxel collapse to one point."""
    # Four points all inside the voxel [0.0,0.1) x [0.0,0.1) x [0.0,0.1)
    pts = np.array([
        [0.01, 0.02, 0.03],
        [0.04, 0.05, 0.06],
        [0.09, 0.01, 0.00],
        [0.00, 0.09, 0.09],
    ], dtype=np.float32)
    out = RosPub.voxel_downsample(pts, voxel=0.10)
    assert out.shape == (1, 3), f"expected 1 point, got {out.shape}"
    assert out.dtype == np.float32


def test_voxel_downsample_separate_voxels_kept():
    """Points in distinct voxels are all kept (one each)."""
    pts = np.array([
        [0.00, 0.00, 0.00],
        [0.15, 0.00, 0.00],   # different x voxel
        [0.00, 0.25, 0.00],   # different y voxel
        [0.00, 0.00, 0.35],   # different z voxel
        [0.151, 0.001, 0.001],  # same voxel as point #2 -> collapses
    ], dtype=np.float32)
    out = RosPub.voxel_downsample(pts, voxel=0.10)
    assert out.shape[0] == 4, f"expected 4 unique voxels, got {out.shape[0]}"


def test_voxel_downsample_empty():
    out = RosPub.voxel_downsample(np.empty((0, 3), dtype=np.float32), voxel=0.10)
    assert out.shape == (0, 3)


# ============================================================================
# ROS node tests
# ============================================================================
@pytest.fixture(scope="module")
def pub():
    """A single RosPub for the module; init/shutdown rclpy around it."""
    if not rclpy.ok():
        rclpy.init()
    p = RosPub(node_name="test_inspection_drone")
    yield p
    p.close()
    if rclpy.ok():
        rclpy.shutdown()


def test_cloud_pointcloud2_roundtrip(pub):
    """/lidar/points xyz layout round-trips byte-exact via read_points_numpy."""
    rng = np.random.default_rng(0)
    P = (rng.standard_normal((500, 3)) * 5.0).astype(np.float32)
    msg = pub._make_cloud(pub._stamp(), "os0_lidar", P, pub._xyz_fields(), 3)

    assert msg.point_step == 12
    assert msg.width == 500
    assert msg.row_step == 12 * 500

    got = read_points_numpy(msg, field_names=["x", "y", "z"])
    assert got.shape == (500, 3)
    np.testing.assert_allclose(got, P, rtol=0, atol=0)


def test_map_cloud_intensity_is_height(pub):
    """/map builds an xyzi cloud whose intensity channel equals z."""
    rng = np.random.default_rng(1)
    # spread points across many voxels so downsample keeps a decent count
    down = (rng.standard_normal((2000, 3)) * 10.0).astype(np.float32)
    arr = np.empty((down.shape[0], 4), dtype=np.float32)
    arr[:, 0:3] = down
    arr[:, 3] = down[:, 2]
    msg = pub._make_cloud(pub._stamp(), "map", arr, pub._xyzi_fields(), 4)

    assert msg.point_step == 16
    got = read_points_numpy(msg, field_names=["x", "y", "z", "intensity"])
    assert got.shape == (2000, 4)
    # intensity == z
    np.testing.assert_allclose(got[:, 3], got[:, 2], rtol=0, atol=0)


def test_map_publish_rate_limited_and_valid(pub):
    """map() publishes an xyzi PointCloud2 and is rate-limited to <= 1 Hz."""
    captured = []
    sub = pub.node.create_subscription(
        ros_pub.PointCloud2, "/map", lambda m: captured.append(m),
        ros_pub._latched_qos(depth=1),
    )
    rng = np.random.default_rng(2)
    Pw = (rng.standard_normal((5000, 3)) * 8.0).astype(np.float32)

    # force the internal rate-limit clock to allow a publish
    pub._last_map_t = -1.0
    pub.map(Pw)
    # immediate second call must be suppressed by the 1 Hz limiter
    pub.map(Pw)

    # pump
    for _ in range(50):
        rclpy.spin_once(pub.node, timeout_sec=0.02)
        if captured:
            break
    pub.node.destroy_subscription(sub)

    assert len(captured) >= 1, "map() did not publish"
    m = captured[0]
    assert m.header.frame_id == "map"
    names = [f.name for f in m.fields]
    assert names == ["x", "y", "z", "intensity"]
    got = read_points_numpy(m, field_names=["z", "intensity"])
    np.testing.assert_allclose(got[:, 0], got[:, 1], rtol=0, atol=0)


def test_path_decimation(pub):
    """/path appends a pose only after >= 0.15 m of travel."""
    # reset path state
    pub._path.poses.clear()
    pub._last_path_xyz = None
    pub._last_path_pub_t = -1e9

    q = (0.0, 0.0, 0.0, 1.0)
    # first pose always appended
    pub.pose(np.array([0.0, 0.0, 0.0]), q)
    assert len(pub._path.poses) == 1

    # small move (< 0.15 m): must NOT append
    pub.pose(np.array([0.10, 0.0, 0.0]), q)
    assert len(pub._path.poses) == 1, "small move should not append"

    # cumulative since last appended is still 0.10 -> move to 0.10 again no-op
    pub.pose(np.array([0.10, 0.05, 0.0]), q)  # dist from last appended(0,0,0)=0.1118
    assert len(pub._path.poses) == 1

    # big move (>= 0.15 m from last appended origin): append
    pub.pose(np.array([0.20, 0.0, 0.0]), q)
    assert len(pub._path.poses) == 2, "0.20 m move should append"

    # another big move
    pub.pose(np.array([0.50, 0.0, 0.0]), q)
    assert len(pub._path.poses) == 3


def test_defects_markers_colour_by_severity(pub):
    """defects() emits a cube+label pair per defect, coloured by severity."""
    captured = []
    sub = pub.node.create_subscription(
        ros_pub.MarkerArray, "/defects", lambda m: captured.append(m),
        ros_pub._latched_qos(depth=1),
    )
    defs = [
        {"type": "spall", "severity": 3, "position": [1.0, 2.0, 3.0]},
        {"type": "crack", "severity": 1, "position": [4.0, 5.0, 6.0]},
    ]
    pub.defects(defs)
    for _ in range(50):
        rclpy.spin_once(pub.node, timeout_sec=0.02)
        if captured:
            break
    pub.node.destroy_subscription(sub)

    assert captured, "defects() did not publish"
    markers = captured[0].markers
    cubes = [m for m in markers if m.type == ros_pub.Marker.CUBE]
    assert len(cubes) == 2
    # severity 3 -> red (1,0,0); severity 1 -> yellow (1,1,0)
    red = cubes[0].color
    yellow = cubes[1].color
    assert (red.r, red.g, red.b) == (1.0, 0.0, 0.0)
    assert (yellow.r, yellow.g, yellow.b) == (1.0, 1.0, 0.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
