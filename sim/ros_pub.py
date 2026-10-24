"""Non-blocking ROS 2 publisher for the Charles River Dam drone inspection sim.

This module is called from inside the Isaac Sim main loop (sim/charles_dam_world.py),
so every public method is designed to be *cheap* and *non-blocking*:

  * no background spin threads that hold the GIL for long -- the caller pumps the
    executor itself by calling :meth:`RosPub.spin`, which does
    ``rclpy.spin_once(node, timeout_sec=0.0)``;
  * PointCloud2 payloads are built straight from numpy buffers (``tobytes``), with
    no per-point Python loops;
  * the heavy, high-rate topics (``/path``, ``/map`` and the image topics) are
    rate-limited internally so a fast render loop cannot flood the graph.

Only numpy, rclpy and the standard ROS message packages are used.

Frames published:  map -> odom -> base_link -> {os0_lidar, gimbal_cam, thermal_cam}.
``map -> odom`` is a static identity transform published once (TRANSIENT_LOCAL) at
construction; ``odom -> base_link`` is dynamic (from :meth:`pose`); sensor frames
are dynamic (from :meth:`sensor_tf`).
"""

from __future__ import annotations

import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import String, Header, ColorRGBA
from geometry_msgs.msg import (
    TransformStamped,
    Transform,
    Vector3,
    Quaternion,
    Point,
    Pose,
    PoseStamped,
)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField, Image
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


# --- QoS presets ------------------------------------------------------------
def _latched_qos(depth: int = 1) -> QoSProfile:
    """TRANSIENT_LOCAL + RELIABLE so late-joining subscribers (RViz started
    after the publisher) still receive the last message."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _sensor_qos(depth: int = 1) -> QoSProfile:
    """BEST_EFFORT, shallow -- for high-rate streaming sensor data."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


# severity -> (r, g, b) for defect cubes: red / orange / yellow.
_SEVERITY_RGB = {
    3: (1.0, 0.0, 0.0),   # red    -- worst
    2: (1.0, 0.5, 0.0),   # orange
    1: (1.0, 1.0, 0.0),   # yellow -- mildest
}


def _severity_to_int(sev) -> int:
    """Coerce a severity (int or str) into {1,2,3}; unknown -> 2 (orange)."""
    if isinstance(sev, (int, float)):
        return int(max(1, min(3, round(sev))))
    s = str(sev).strip().lower()
    table = {
        "low": 1, "minor": 1, "1": 1, "yellow": 1,
        "med": 2, "medium": 2, "moderate": 2, "2": 2, "orange": 2,
        "high": 3, "severe": 3, "critical": 3, "3": 3, "red": 3,
    }
    return table.get(s, 2)


class RosPub:
    """ROS 2 publishing facade for the inspection drone.

    Parameters
    ----------
    node_name:
        Name of the underlying rclpy node.
    """

    def __init__(self, node_name: str = "inspection_drone"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)

        # --- publishers -----------------------------------------------------
        self._pub_tf = self.node.create_publisher(TFMessage, "/tf", 20)
        self._pub_tf_static = self.node.create_publisher(
            TFMessage, "/tf_static", _latched_qos(depth=1)
        )
        self._pub_odom = self.node.create_publisher(Odometry, "/odom", 10)
        self._pub_path = self.node.create_publisher(Path, "/path", 1)
        self._pub_map = self.node.create_publisher(
            PointCloud2, "/map", _latched_qos(depth=1)
        )
        self._pub_mission = self.node.create_publisher(
            Path, "/mission", _latched_qos(depth=1)
        )
        self._pub_mission_mk = self.node.create_publisher(
            MarkerArray, "/mission/markers", _latched_qos(depth=1)
        )
        self._pub_defects = self.node.create_publisher(
            MarkerArray, "/defects", _latched_qos(depth=1)
        )
        self._pub_status = self.node.create_publisher(String, "/drone/status", 10)

        # per-topic PointCloud2 / Image publishers created lazily
        self._cloud_pubs: dict[str, "rclpy.publisher.Publisher"] = {}
        self._image_pubs: dict[str, "rclpy.publisher.Publisher"] = {}

        # --- /path accumulation + decimation state --------------------------
        self._path = Path()
        self._path.header.frame_id = "map"
        self._last_path_xyz: np.ndarray | None = None
        self._last_path_pub_t = -1.0          # last time /path was published
        self._path_min_move = 0.15            # m, min travel to append a pose
        self._path_pub_period = 0.2           # s, publish /path at most 5 Hz

        # --- rate-limit bookkeeping -----------------------------------------
        self._last_map_t = -1.0               # /map at most 1 Hz
        self._map_period = 1.0
        self._last_image_t: dict[str, float] = {}
        self._image_period = 0.1              # 10 Hz per topic

        # --- static map->odom identity transform (latched) ------------------
        self._publish_static_map_odom()

    # ------------------------------------------------------------------ time
    def _stamp(self) -> TimeMsg:
        return self.node.get_clock().now().to_msg()

    def _now_sec(self) -> float:
        t = self.node.get_clock().now()
        return t.nanoseconds * 1e-9

    # ------------------------------------------------------------------- TF
    @staticmethod
    def _make_tf(stamp, parent, child, pos, quat_xyzw) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat_xyzw[0])
        t.transform.rotation.y = float(quat_xyzw[1])
        t.transform.rotation.z = float(quat_xyzw[2])
        t.transform.rotation.w = float(quat_xyzw[3])
        return t

    def _publish_static_map_odom(self) -> None:
        tf = self._make_tf(
            self._stamp(), "map", "odom", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        )
        self._pub_tf_static.publish(TFMessage(transforms=[tf]))

    def sensor_tf(self, child, pos, quat_xyzw, parent="base_link") -> None:
        """Publish a dynamic TF for a sensor frame relative to ``parent``."""
        tf = self._make_tf(self._stamp(), parent, child, pos, quat_xyzw)
        self._pub_tf.publish(TFMessage(transforms=[tf]))

    # ------------------------------------------------------------------ pose
    def pose(self, pos, quat_xyzw, vel=None) -> None:
        """Publish odom->base_link TF, /odom and (decimated) /path.

        ``pos`` / ``vel`` are length-3 world-frame numpy arrays; ``quat_xyzw``
        is ``(x, y, z, w)``.
        """
        stamp = self._stamp()
        pos = np.asarray(pos, dtype=float).reshape(3)

        # dynamic TF odom -> base_link
        tf = self._make_tf(stamp, "odom", "base_link", pos, quat_xyzw)
        self._pub_tf.publish(TFMessage(transforms=[tf]))

        # odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = float(quat_xyzw[0])
        odom.pose.pose.orientation.y = float(quat_xyzw[1])
        odom.pose.pose.orientation.z = float(quat_xyzw[2])
        odom.pose.pose.orientation.w = float(quat_xyzw[3])
        if vel is not None:
            vel = np.asarray(vel, dtype=float).reshape(3)
            odom.twist.twist.linear.x = float(vel[0])
            odom.twist.twist.linear.y = float(vel[1])
            odom.twist.twist.linear.z = float(vel[2])
        self._pub_odom.publish(odom)

        # /path: append only if we moved >= min_move since the last appended pose
        if (self._last_path_xyz is None
                or np.linalg.norm(pos - self._last_path_xyz) >= self._path_min_move):
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = "map"
            ps.pose.position.x = float(pos[0])
            ps.pose.position.y = float(pos[1])
            ps.pose.position.z = float(pos[2])
            ps.pose.orientation.w = 1.0
            self._path.poses.append(ps)
            self._last_path_xyz = pos.copy()

            now = self._now_sec()
            if now - self._last_path_pub_t >= self._path_pub_period:
                self._path.header.stamp = stamp
                self._pub_path.publish(self._path)
                self._last_path_pub_t = now

    # ---------------------------------------------------------------- clouds
    @staticmethod
    def _xyz_fields() -> list:
        return [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    @staticmethod
    def _xyzi_fields() -> list:
        return [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

    def _make_cloud(self, stamp, frame, arr: np.ndarray, fields, ncols: int) -> PointCloud2:
        """Build a PointCloud2 from a C-contiguous float32 array of shape
        ``(N, ncols)`` straight from its byte buffer (no Python per-point loop)."""
        arr = np.ascontiguousarray(arr, dtype=np.float32)
        n = int(arr.shape[0])
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=frame)
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 4 * ncols
        msg.row_step = msg.point_step * n
        msg.is_dense = True
        msg.data = arr.tobytes()
        return msg

    def cloud(self, P, frame="os0_lidar", topic="/lidar/points") -> None:
        """Publish an ``(N, 3)`` float32 cloud (sensor frame) as PointCloud2."""
        P = np.asarray(P, dtype=np.float32).reshape(-1, 3)
        pub = self._cloud_pubs.get(topic)
        if pub is None:
            pub = self.node.create_publisher(PointCloud2, topic, _sensor_qos(depth=1))
            self._cloud_pubs[topic] = pub
        msg = self._make_cloud(self._stamp(), frame, P, self._xyz_fields(), 3)
        pub.publish(msg)

    # --- voxel key packing -------------------------------------------------
    # Pack a 3-D integer voxel index into a single int64 so we can use the fast
    # 1-D ``np.unique`` instead of ``np.unique(axis=0)`` (row-wise, much slower).
    # 21 bits per axis gives a signed range of +-2**20 voxels; at a 0.10 m voxel
    # that is +-104 km per axis -- far beyond any sim world.
    _VK_BITS = 21
    _VK_OFFSET = 1 << (_VK_BITS - 1)          # 2**20, shift signed -> unsigned
    _VK_MASK = (1 << _VK_BITS) - 1

    @classmethod
    def _pack_voxel_keys(cls, Pw: np.ndarray, voxel: float) -> np.ndarray:
        """Return an int64 array of packed voxel keys for points ``Pw``."""
        ijk = np.floor(np.asarray(Pw, dtype=np.float64) / voxel).astype(np.int64)
        ijk += cls._VK_OFFSET
        np.clip(ijk, 0, cls._VK_MASK, out=ijk)
        b = cls._VK_BITS
        return (ijk[:, 0] << (2 * b)) | (ijk[:, 1] << b) | ijk[:, 2]

    @classmethod
    def voxel_downsample(cls, Pw: np.ndarray, voxel: float = 0.10) -> np.ndarray:
        """Voxel-downsample world points to ``voxel`` m using integer voxel
        keys. Returns one representative point per voxel (the first-seen point),
        in first-occurrence order.

        Uses a packed-int64 key + 1-D ``np.unique`` (far faster than the
        row-wise ``np.unique(axis=0)``) while preserving identical behaviour.

        Kept as a staticmethod-like classmethod so tests can exercise it
        directly.
        """
        Pw = np.asarray(Pw, dtype=np.float64).reshape(-1, 3)
        if Pw.shape[0] == 0:
            return Pw.astype(np.float32)
        keys = cls._pack_voxel_keys(Pw, voxel)
        # unique keys -> indices of first occurrence, then restore input order
        _, idx = np.unique(keys, return_index=True)
        idx.sort()
        return Pw[idx].astype(np.float32)

    def _reset_map_state(self) -> None:
        """Clear the incremental /map accumulator (used when the caller's point
        buffer shrinks / restarts)."""
        self._map_n_seen = 0                      # points of Pw already folded in
        self._map_seen_keys = np.empty(0, dtype=np.int64)   # SORTED kept keys
        self._map_rows_list: list = []            # list of float32 (k,3) blocks
        self._map_n_rows = 0                      # total kept representatives
        self._map_data = bytearray()              # packed xyzi bytes (uncapped)
        self._map_cloud_cache = None              # cached (n, bytes) for publish

    def map_extend(self, new_pts) -> None:
        """Add only the NEW world-frame points (caller keeps no buffer); folded into /map at most 1 Hz."""
        if not hasattr(self, "_map_pend"):
            self._map_pend = []
        self._map_pend.append(np.asarray(new_pts, dtype=np.float32).reshape(-1, 3))
        if self._now_sec() - self._last_map_t < self._map_period:
            return
        new = np.concatenate(self._map_pend, axis=0); self._map_pend = []
        if not hasattr(self, "_map_n_seen"):
            self._reset_map_state()
        self._map_n_seen = 0                  # map() then treats `new` as the unseen slice
        self._last_map_t = -1e9
        self.map(new)

    def map(self, Pw) -> None:
        """Publish the accumulated world-frame map (voxel-downsampled, capped,
        coloured by height). Rate-limited to 1 Hz internally.

        The caller passes the whole accumulated world-point buffer every call;
        the unchanged prefix (detected by length) is not reprocessed. Only
        ``Pw[n_seen:]`` is voxel-folded into an incremental voxel set, and the
        packed xyzi byte buffer is *extended* (not rebuilt) with the newly-seen
        voxels, so a steady-state render frame only pays for its handful of new
        points.

        The result matches downsampling the whole buffer fresh: the kept
        representative per voxel is the first point of the full buffer in that
        voxel, and voxels appear in first-occurrence order (then capped to
        400k by uniform subsample, identical to the original).
        """
        now = self._now_sec()
        if now - self._last_map_t < self._map_period:
            return
        self._last_map_t = now

        Pw = np.asarray(Pw, dtype=np.float32).reshape(-1, 3)
        total = Pw.shape[0]

        if not hasattr(self, "_map_n_seen"):
            self._reset_map_state()
        # If the buffer shrank (restart) the unchanged-prefix assumption breaks.
        if total < self._map_n_seen:
            self._reset_map_state()

        new = Pw[self._map_n_seen:]
        changed = False
        if new.shape[0] > 0:
            # Unique voxels within the new slice, in first-occurrence order.
            keys = self._pack_voxel_keys(new, 0.10)
            uniq, first_idx = np.unique(keys, return_index=True)
            first_idx.sort()                       # -> first-occurrence order
            cand_keys = keys[first_idx]            # new-slice keys, first-occ order
            cand_rows = new[first_idx]

            # Drop voxels already kept. Membership is tested against the sorted
            # running key set with np.searchsorted -- O(M log K) over only the
            # small new slice (M), never a per-point Python scan.
            seen = self._map_seen_keys
            if seen.shape[0]:
                pos = np.searchsorted(seen, cand_keys)
                np.clip(pos, 0, seen.shape[0] - 1, out=pos)
                fresh_mask = seen[pos] != cand_keys
                add_keys = cand_keys[fresh_mask]
                add_rows = cand_rows[fresh_mask]
            else:
                add_keys = cand_keys               # cold fold: all fresh
                add_rows = cand_rows

            if add_rows.shape[0]:
                # merge new keys into the sorted set (one sort of K+M keys)
                self._map_seen_keys = np.sort(
                    np.concatenate([seen, add_keys])
                )
                self._map_rows_list.append(add_rows)
                self._map_n_rows += add_rows.shape[0]

                # Extend the packed xyzi byte buffer with the new rows only.
                k = add_rows.shape[0]
                block = np.empty((k, 4), dtype=np.float32)
                block[:, 0:3] = add_rows
                block[:, 3] = add_rows[:, 2]       # intensity = z
                self._map_data += np.ascontiguousarray(block).tobytes()
                changed = True
            self._map_n_seen = total

        n_rows = self._map_n_rows

        # (Re)materialise the publish buffer only when the voxel set changed.
        if self._map_cloud_cache is None or changed:
            if n_rows > 400_000:
                # Rare: cap by uniform subsample (matches the original).
                rows = np.concatenate(self._map_rows_list, axis=0)
                sel = np.linspace(0, n_rows - 1, 400_000).astype(np.int64)
                d = rows[sel]
                arr = np.empty((400_000, 4), dtype=np.float32)
                arr[:, 0:3] = d
                arr[:, 3] = d[:, 2]
                self._map_cloud_cache = (
                    400_000, np.ascontiguousarray(arr).tobytes()
                )
            else:
                # Under the cap: the extended byte buffer is the payload.
                self._map_cloud_cache = (n_rows, bytes(self._map_data))

        n, data = self._map_cloud_cache
        msg = PointCloud2()
        msg.header = Header(stamp=self._stamp(), frame_id="map")
        msg.height = 1
        msg.width = n
        msg.fields = self._xyzi_fields()
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = data
        self._pub_map.publish(msg)

    # ----------------------------------------------------------------- images
    def image(self, topic, img, frame, encoding="rgb8") -> None:
        """Publish an Image (HxWx3 uint8 rgb8, or HxW float32 32FC1).
        Rate-limited per topic to 10 Hz."""
        now = self._now_sec()
        last = self._last_image_t.get(topic, -1.0)
        if now - last < self._image_period:
            return
        self._last_image_t[topic] = now

        pub = self._image_pubs.get(topic)
        if pub is None:
            pub = self.node.create_publisher(Image, topic, _sensor_qos(depth=1))
            self._image_pubs[topic] = pub

        img = np.asarray(img)
        msg = Image()
        msg.header = Header(stamp=self._stamp(), frame_id=frame)
        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        if encoding == "32FC1":
            arr = np.ascontiguousarray(img, dtype=np.float32)
            msg.step = msg.width * 4
            msg.data = arr.tobytes()
        else:  # rgb8 / bgr8 / mono8 etc.
            arr = np.ascontiguousarray(img, dtype=np.uint8)
            channels = 1 if arr.ndim == 2 else arr.shape[2]
            msg.step = msg.width * channels
            msg.data = arr.tobytes()
        pub.publish(msg)

    # ---------------------------------------------------------------- mission
    def mission(self, waypoints) -> None:
        """Publish the planned mission as a Path (/mission) and a MarkerArray
        (/mission/markers). ``waypoints`` is a list of
        ``(pos(3), target(3), element_name_str)``. Both topics are latched."""
        stamp = self._stamp()

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "map"

        markers = MarkerArray()

        # LINE_STRIP through all waypoints
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = "map"
        line.ns = "mission_line"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        line.pose.orientation.w = 1.0

        prev_element = None
        for i, (pos, target, element) in enumerate(waypoints):
            pos = np.asarray(pos, dtype=float).reshape(3)

            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = "map"
            ps.pose.position.x = float(pos[0])
            ps.pose.position.y = float(pos[1])
            ps.pose.position.z = float(pos[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

            pt = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            line.points.append(pt)

            # sphere at each waypoint
            sph = Marker()
            sph.header.stamp = stamp
            sph.header.frame_id = "map"
            sph.ns = "mission_wp"
            sph.id = i
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position = pt
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.25
            sph.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
            markers.markers.append(sph)

            # TEXT label at the first waypoint of each new element
            if element != prev_element:
                txt = Marker()
                txt.header.stamp = stamp
                txt.header.frame_id = "map"
                txt.ns = "mission_label"
                txt.id = i
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position = Point(
                    x=float(pos[0]), y=float(pos[1]), z=float(pos[2]) + 0.6
                )
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.5
                txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                txt.text = str(element)
                markers.markers.append(txt)
                prev_element = element

        markers.markers.append(line)

        self._pub_mission.publish(path)
        self._pub_mission_mk.publish(markers)

    # ---------------------------------------------------------------- defects
    def defects(self, list_of_dicts) -> None:
        """Publish defects as a latched MarkerArray on /defects.

        Each dict must carry a position (``position`` or ``pos``), a severity
        (``severity``, int or str) and a kind (``kind``/``type``). A cube
        coloured by severity (red/orange/yellow) plus a text label
        ``"kind (severity)"`` is emitted per defect.
        """
        stamp = self._stamp()
        markers = MarkerArray()

        for i, d in enumerate(list_of_dicts):
            pos = d.get("position", d.get("pos", [0.0, 0.0, 0.0]))
            pos = np.asarray(pos, dtype=float).reshape(3)
            kind = d.get("kind", d.get("type", "defect"))
            sev_raw = d.get("severity", 2)
            sev = _severity_to_int(sev_raw)
            r, g, b = _SEVERITY_RGB[sev]

            cube = Marker()
            cube.header.stamp = stamp
            cube.header.frame_id = "map"
            cube.ns = "defect"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = cube.scale.z = 0.5
            cube.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            markers.markers.append(cube)

            txt = Marker()
            txt.header.stamp = stamp
            txt.header.frame_id = "map"
            txt.ns = "defect_label"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = Point(
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2]) + 0.5
            )
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f"{kind} ({sev_raw})"
            markers.markers.append(txt)

        self._pub_defects.publish(markers)

    # ---------------------------------------------------------------- status
    def status(self, text) -> None:
        """Publish a std_msgs/String on /drone/status (GNSS / fix / mode text)."""
        self._pub_status.publish(String(data=str(text)))

    # ------------------------------------------------------------------ spin
    def spin(self) -> None:
        """Pump the executor once, non-blocking."""
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        """Destroy the node. ``rclpy.shutdown()`` is left to the application so
        other nodes in the same process are not affected."""
        try:
            self.node.destroy_node()
        except Exception:
            pass
