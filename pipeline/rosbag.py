"""Real ROS2 rosbag2 (.mcap) ingest for a drone inspection mission.

Decodes an actual bag -- the one recorded from Isaac Sim + ArduCopter SITL --
rather than a JSON fixture. It:

  - reads /odom into a pose timeline (map/odom frame),
  - reads /lidar/points into retrace Captures (kept light: offsets, not points,
    are stored; the cloud is only materialised when a detection retraces it),
  - reads /camera/rgb timestamps as the frames detection runs over,
  - turns the mission's ground-truth defects (defects_gt.json) into drone
    detections attached to the nearest-in-time RGB frame.

Ground truth stands in for the camera model here: this bag carries raw sensor
data and no trained YOLO weights are installed, so GT is the honest detection
source for an end-to-end test (the architecture anticipates synthetic GT for
exactly this). Swap in CameraDetector over the real frames once weights exist.

Needs: mcap, mcap-ros2-support, zstandard, numpy (see requirements.txt).
Topic names are configurable so other bags with different topics still work.
"""
from __future__ import annotations

import bisect
import json
from pathlib import Path
from typing import Optional

from .indexing import Capture
from .schema import Pose, make_detection

DEFAULT_TOPICS = {
    "rgb": "/camera/rgb",
    "lidar": "/lidar/points",
    "odom": "/odom",
}

# Map the bag's defect taxonomy to the pipeline's drone classes.
GT_CLASS_MAP = {
    "spall": "spall",
    "crack": "crack",
    "efflorescence": "efflorescence",
    "rust_streak": "rust_streak",
    "exposed_rebar": "exposed_rebar",
    "corrosion": "corrosion_stain",
    "corrosion_stain": "corrosion_stain",
}

# GT severity (1-3) -> a plausible detector confidence, so low-severity defects
# don't all read as max-confidence. Code assigns the final severity later.
GT_SEVERITY_CONF = {1: 0.55, 2: 0.72, 3: 0.9}


class PoseTimeline:
    """Odometry poses sorted by time; nearest-pose lookup for any timestamp."""

    def __init__(self):
        self._t: list[float] = []
        self._p: list[Pose] = []

    def add(self, t: float, pose: Pose) -> None:
        i = bisect.bisect_left(self._t, t)
        self._t.insert(i, t)
        self._p.insert(i, pose)

    def at(self, t: float) -> Pose:
        if not self._t:
            return Pose()
        i = bisect.bisect_left(self._t, t)
        best = min((j for j in (i - 1, i, i + 1) if 0 <= j < len(self._t)),
                   key=lambda j: abs(self._t[j] - t), default=0)
        return self._p[best]

    def span(self):
        return (self._t[0], self._t[-1]) if self._t else (None, None)

    def __len__(self):
        return len(self._t)


def _stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec / 1e9


def decode_bag(bag_dir: str, topics: Optional[dict] = None) -> tuple[list[Capture], PoseTimeline]:
    """Decode the bag into (captures, pose_timeline).

    Captures of kind 'rgb' and 'lidar' get their pose joined from /odom.
    Heavy pixel/point payloads are NOT stored; lidar captures keep the message
    offset so the LiDAR stage can read just that cloud when it is retraced.
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    topics = {**DEFAULT_TOPICS, **(topics or {})}
    mcap_path = _find_mcap(bag_dir)

    pose_tl = PoseTimeline()
    rgb_msgs: list[tuple[float, str]] = []     # (time, capture_id)
    lidar_msgs: list[tuple[float, str, int]] = []  # (time, capture_id, offset)

    # Only /odom needs full deserialization. For rgb/lidar we just need each
    # message's time + sequence, so we skip decoding those heavy payloads
    # (1056 images + 4288 clouds) entirely -- a large speedup on real bags.
    decoder = DecoderFactory()
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        odom_decode = None
        for schema, channel, message in reader.iter_messages(
                topics=[topics["odom"], topics["rgb"], topics["lidar"]]):
            topic = channel.topic
            t = message.log_time / 1e9
            if topic == topics["odom"]:
                if odom_decode is None:
                    odom_decode = decoder.decoder_for(channel.message_encoding, schema)
                ros_msg = odom_decode(message.data)
                p = ros_msg.pose.pose
                o = p.orientation
                pose_tl.add(_stamp_s(ros_msg.header) or t,
                            Pose(x=p.position.x, y=p.position.y, z=p.position.z,
                                 roll=o.x, pitch=o.y, yaw=o.z))   # quat stored raw
            elif topic == topics["rgb"]:
                rgb_msgs.append((t, f"rgb:{message.sequence}"))
            elif topic == topics["lidar"]:
                lidar_msgs.append((t, f"lidar:{message.sequence}", message.log_time))

    captures: list[Capture] = []
    for t, cid in rgb_msgs:
        captures.append(Capture("rgb", t, cid, pose_tl.at(t), payload={}))
    for t, cid, off in lidar_msgs:
        captures.append(Capture("lidar", t, cid, pose_tl.at(t),
                                payload={"bag": str(mcap_path), "log_time": off}))
    return captures, pose_tl


def _find_mcap(bag_dir: str) -> Path:
    d = Path(bag_dir)
    if d.is_file() and d.suffix == ".mcap":
        return d
    mcaps = sorted(d.glob("*.mcap"))
    if not mcaps:
        raise FileNotFoundError(f"no .mcap found in {bag_dir}")
    return mcaps[0]


def gt_detections(bag_dir: str, mission_id: str, captures: list[Capture],
                  pose_tl: PoseTimeline):
    """Build drone detections from defects_gt.json, each attached to the
    nearest RGB frame in space (the frame whose pose is closest to the defect's
    world position). This mirrors how a camera detection would arise: a defect
    seen in a frame captured near it."""
    gt_path = Path(bag_dir) / "defects_gt.json"
    if not gt_path.exists():
        return []
    gt = json.loads(gt_path.read_text(encoding="utf-8-sig"))
    rgb_caps = [c for c in captures if c.kind == "rgb"]
    detections = []
    for d in gt.get("defects", []):
        cls = GT_CLASS_MAP.get(d.get("type"))
        if cls is None:
            continue
        pos = d.get("position", [0, 0, 0])
        frame = _nearest_frame(rgb_caps, pos)
        if frame is None:
            continue
        conf = GT_SEVERITY_CONF.get(int(d.get("severity", 2)), 0.7)
        det = make_detection(
            mission_id=mission_id, platform="drone", cls=cls, confidence=conf,
            timestamp_utc=frame.timestamp_utc, pose=frame.pose,
            source_frame_id=frame.capture_id, evidence_frame_ids=[frame.capture_id],
            model_version="ground-truth@defects_gt",
        )
        # Keep the defect's world position for the element map / geometry region.
        det.location.geodetic.lat = None
        det._gt_position = pos          # type: ignore[attr-defined]
        det._gt_element = d.get("element", "")  # type: ignore[attr-defined]
        det._gt_severity = int(d.get("severity", 0))  # type: ignore[attr-defined]
        detections.append(det)
    return detections


def _nearest_frame(rgb_caps: list[Capture], pos) -> Optional[Capture]:
    best, best_d = None, float("inf")
    for c in rgb_caps:
        dx = c.pose.x - pos[0]
        dy = c.pose.y - pos[1]
        dz = c.pose.z - pos[2]
        dsq = dx * dx + dy * dy + dz * dz
        if dsq < best_d:
            best, best_d = c, dsq
    return best


def detect_inspection_frames(bag_dir: str, mission_id: str, camera,
                             conf_threshold: float = 0.25):
    """Run the REAL camera detector over the per-waypoint stills in
    inspection_frames/ (NNNN.png + NNNN.json), using each frame's own drone pose
    and element label. This is genuine model inference -- not ground truth.

    Returns (detections, stats) where stats summarises frames/with-defects.
    """
    import json
    from pathlib import Path

    frames_dir = Path(bag_dir) / "inspection_frames"
    pngs = sorted(frames_dir.glob("[0-9][0-9][0-9][0-9].png"))
    detections = []
    frames_with = 0
    for png in pngs:
        sidecar = png.with_suffix(".json")
        meta = json.loads(sidecar.read_text(encoding="utf-8-sig")) if sidecar.exists() else {}
        element = meta.get("element", "")
        if element in ("transit", "", None):
            continue  # transit frames aren't aimed at a structure element
        drone = meta.get("drone", [0, 0, 0])
        pose = Pose(x=drone[0], y=drone[1], z=drone[2])
        cap = Capture("rgb", float(meta.get("i", 0)), f"frame:{png.stem}", pose,
                      payload={"image_path": str(png)})
        dets = camera.detect(mission_id, cap)
        if dets:
            frames_with += 1
        # Approximate the defect's world position: project from the drone toward
        # the camera target so it lands on the structure surface (standoff away),
        # rather than using the aim-centre target point (which sits off-surface).
        surface_pos = _surface_point(drone, meta.get("target"), meta.get("standoff_m"))
        for det in dets:
            det._frame_element = element      # type: ignore[attr-defined]
            det._frame_target = meta.get("target")   # type: ignore[attr-defined]
            det._surface_pos = surface_pos    # type: ignore[attr-defined]
            det._gnss = meta.get("gnss")      # type: ignore[attr-defined]
            detections.append(det)
    stats = {"frames_total": len(pngs), "frames_inspected": len(pngs),
             "frames_with_defects": frames_with, "detections": len(detections)}
    return detections, stats


def _surface_point(drone, target, standoff):
    """Point `standoff` metres from the drone along the drone->target ray. This
    approximates where the defect sits on the structure surface, since the
    camera target is the aim centre (often slightly off the surface)."""
    import math
    if not (drone and target and standoff):
        return target or drone
    d = [float(x) for x in drone]
    t = [float(x) for x in target]
    v = [t[0] - d[0], t[1] - d[1], t[2] - d[2]]
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) or 1.0
    u = [c / n for c in v]
    return [round(d[i] + u[i] * float(standoff), 3) for i in range(3)]
