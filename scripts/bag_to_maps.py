"""Rebuild the 3D maps from a recorded inspection bag.

    source /opt/ros/jazzy/setup.bash
    python3 scripts/bag_to_maps.py bags/run_<time>      -> bags/run_<time>/map/{drone_lidar,rov_sonar,combined}_world.{pcd,ply,npy}

Drone lidar: /lidar/points (os0_lidar frame) placed with the TF chain odom -> base_link -> os0_lidar (nearest stamps).
ROV sonar: /rov/sonar/points are already in the map frame (from the ROV's true pose). 5 cm voxels.
"""
import os, sys, bisect
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from sensor_msgs_py import point_cloud2 as pc2


def tmat(tr):
    q = tr.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def voxel(P, v=0.05):
    k = np.floor(P / v).astype(np.int64); _, i = np.unique(k, axis=0, return_index=True)
    return P[np.sort(i)]


def save(base, P, color):
    P = P.astype(np.float32); np.save(base + ".npy", P)
    with open(base + ".pcd", "wb") as f:
        f.write((f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(P)}\nHEIGHT 1\n"
                 f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(P)}\nDATA binary\n").encode()); f.write(P.tobytes())
    rec = np.empty(len(P), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = P.T; c = color(P); rec["red"], rec["green"], rec["blue"] = c.T
    with open(base + ".ply", "wb") as f:
        f.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {len(P)}\nproperty float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n").encode()); f.write(rec.tobytes())


def height_color(P):
    z = np.clip((P[:, 2] + 1) / 11, 0, 1)
    return np.stack([255 * z, 255 * (1 - abs(2 * z - 1)), 255 * (1 - z)], 1).astype(np.uint8)


def sonar_color(P):
    d = np.clip(-P[:, 2] / 6, 0, 1)
    return np.stack([255 - 60 * d, 170 - 90 * d, 40 + 30 * d], 1).astype(np.uint8)


bag = sys.argv[1].rstrip("/")
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/lidar/points", "/rov/sonar/points"]))
tf = {"base_link": ([], []), "os0_lidar": ([], [])}
lidar, sonar = [], []
while r.has_next():
    topic, data, t = r.read_next()
    if topic == "/tf":
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.child_frame_id in tf:
                tf[tr.child_frame_id][0].append(t); tf[tr.child_frame_id][1].append(tmat(tr.transform))
    elif topic == "/lidar/points":
        lidar.append((t, pc2.read_points_numpy(deserialize_message(data, PointCloud2), field_names=("x", "y", "z"))[::6]))
    else:
        sonar.append(pc2.read_points_numpy(deserialize_message(data, PointCloud2), field_names=("x", "y", "z"))[::2])


def near(child, t):
    ts, Ts = tf[child]; i = min(max(bisect.bisect_left(ts, t), 0), len(ts) - 1)
    return Ts[i]


W = []
for t, P in lidar:
    if not len(P) or not tf["base_link"][0] or not tf["os0_lidar"][0]: continue
    T = near("base_link", t) @ near("os0_lidar", t)
    W.append(P @ T[:3, :3].T + T[:3, 3])
out = os.path.join(bag, "map"); os.makedirs(out, exist_ok=True)
L = voxel(np.concatenate(W)) if W else np.zeros((0, 3)); L = L[L[:, 2] > 0.1]
S = voxel(np.concatenate(sonar)) if sonar else np.zeros((0, 3))
save(os.path.join(out, "drone_lidar_world"), L, height_color)
save(os.path.join(out, "rov_sonar_world"), S, sonar_color)
C = np.concatenate([L, S]); save(os.path.join(out, "combined_world"), C, lambda P: np.where((P[:, 2] < 0)[:, None], sonar_color(P), height_color(P)))
print(f"drone lidar {len(L):,} pts, ROV sonar {len(S):,} pts, combined {len(C):,} (5 cm voxels) -> {out}")
