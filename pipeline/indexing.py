"""INGEST + DECODE + INDEX (pipeline steps 1-2) and RETRACE (step 4).

The drone and ROV record every stream continuously, so the data for every spot
is already in the rosbag .mcap. This module decodes it and builds a TIME + POSE
index so that when a detector flags a defect, we can RETRACE into the recorded
data by (timestamp, pose) and pull the co-located LiDAR cloud or camera frame --
no command ever goes back to a vehicle.

Decoding an actual .mcap needs the `mcap` reader (optionally the ROS2 message
definitions). To keep the pipeline runnable offline with nothing installed --
the repo's standing convention -- the reader degrades to a JSON fixture when the
`mcap` package is absent. The index + retrace logic is identical either way.
"""
from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from .schema import Pose

# Channels we care about, keyed by the stream kind the pipeline consumes.
STREAM_KINDS = ("rgb", "lidar", "sonar", "rov_camera", "pose", "gps")


@dataclass
class Capture:
    """One decoded message from the bag: a frame, scan, cloud or pose sample."""
    kind: str                       # one of STREAM_KINDS
    timestamp_utc: float            # epoch seconds
    capture_id: str                 # frame_id / scan_id
    pose: Pose = field(default_factory=Pose)
    payload: dict = field(default_factory=dict)   # kind-specific (path to cloud, bbox space, etc.)


class TimePoseIndex:
    """Per-kind captures sorted by timestamp, searchable by (timestamp, pose).

    Retrace resolves the co-located capture of another kind: nearest in time,
    then closest in pose within a gate. This is what lets the flow stay one-way.
    """

    def __init__(self, max_dt_s: float = 0.5, max_pose_m: float = 2.0):
        self.max_dt_s = max_dt_s
        self.max_pose_m = max_pose_m
        self._by_kind: dict[str, list[Capture]] = {k: [] for k in STREAM_KINDS}
        self._ts_by_kind: dict[str, list[float]] = {k: [] for k in STREAM_KINDS}

    def add(self, cap: Capture) -> None:
        if cap.kind not in self._by_kind:
            raise ValueError(f"unknown stream kind {cap.kind!r}")
        lst, ts = self._by_kind[cap.kind], self._ts_by_kind[cap.kind]
        pos = bisect.bisect_left(ts, cap.timestamp_utc)
        lst.insert(pos, cap)
        ts.insert(pos, cap.timestamp_utc)

    def count(self, kind: str) -> int:
        return len(self._by_kind.get(kind, []))

    def retrace(self, kind: str, timestamp_utc: float, pose: Optional[Pose] = None) -> Optional[Capture]:
        """Find the recorded `kind` capture co-located with (timestamp, pose).

        Returns None if nothing falls inside the time/pose gates -- e.g. the ROV
        camera frame is too far off in a turbid stretch, so the defect stays
        sonar-only. That is an expected outcome, not an error.
        """
        ts = self._ts_by_kind.get(kind) or []
        if not ts:
            return None
        pos = bisect.bisect_left(ts, timestamp_utc)
        # Consider the neighbours on both sides of the insertion point.
        candidates = []
        for i in (pos - 1, pos, pos + 1):
            if 0 <= i < len(ts):
                dt = abs(ts[i] - timestamp_utc)
                if dt <= self.max_dt_s:
                    candidates.append((dt, self._by_kind[kind][i]))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0])
        if pose is None:
            return candidates[0][1]
        # Prefer the time-closest candidate that is also within the pose gate.
        for _, cap in candidates:
            if _pose_dist(cap.pose, pose) <= self.max_pose_m:
                return cap
        return None


def _pose_dist(a: Pose, b: Pose) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _pose_from(d: dict) -> Pose:
    return Pose(**{k: float(d.get(k, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")})


def decode_mcap(mcap_path: str) -> list[Capture]:
    """Decode a rosbag .mcap into Captures.

    Real path: use the `mcap` reader. Offline/demo path: read a JSON sidecar of
    the same name (``<bag>.json``) or the bag itself if it is already JSON. Both
    produce the same Capture list, so the rest of the pipeline is unchanged.
    """
    p = Path(mcap_path)
    sidecar = p.with_suffix(".json")
    try:
        from mcap.reader import make_reader  # type: ignore
    except ImportError:
        make_reader = None

    if make_reader is not None and p.suffix == ".mcap" and p.exists():
        return _decode_real_mcap(p, make_reader)
    # Fixture fallback keeps the pipeline runnable with nothing installed.
    src = p if (p.exists() and p.suffix == ".json") else sidecar
    if not src.exists():
        raise FileNotFoundError(
            f"no .mcap reader available and no JSON fixture found at {src}. "
            f"Install the `mcap` package for real bags or provide a JSON sidecar."
        )
    raw = json.loads(src.read_text(encoding="utf-8-sig"))
    return [_capture_from_json(m) for m in raw.get("captures", [])]


def _capture_from_json(m: dict) -> Capture:
    return Capture(
        kind=m["kind"],
        timestamp_utc=float(m["timestamp_utc"]),
        capture_id=str(m.get("capture_id", "")),
        pose=_pose_from(m.get("pose", {})),
        payload=m.get("payload", {}),
    )


# Map ROS2 topics to the pipeline's stream kinds. Topic names are the common
# conventions; adapt to the actual bag's topics via config if they differ.
TOPIC_KIND = {
    "/drone/camera/image_raw": "rgb",
    "/drone/lidar/points": "lidar",
    "/drone/gps/fix": "gps",
    "/drone/pose": "pose",
    "/rov/sonar/image": "sonar",
    "/rov/camera/image_raw": "rov_camera",
    "/rov/pose": "pose",
}


def _decode_real_mcap(path: Path, make_reader) -> list[Capture]:
    """Decode a genuine .mcap. Payloads keep a reference to the message, not the
    raw pixels/points, so the index stays light; heavy data is loaded lazily by
    the detection/LiDAR stages when a capture is actually retraced."""
    captures: list[Capture] = []
    with path.open("rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            kind = TOPIC_KIND.get(channel.topic)
            if kind is None:
                continue
            ts = message.log_time / 1e9   # ns -> s
            captures.append(Capture(
                kind=kind,
                timestamp_utc=ts,
                capture_id=f"{channel.topic}:{message.sequence}",
                pose=Pose(),  # pose is joined from /pose + /gps messages downstream
                payload={"topic": channel.topic, "sequence": message.sequence,
                         "schema": getattr(schema, "name", "")},
            ))
    return captures


def build_index(captures: list[Capture], max_dt_s: float = 0.5, max_pose_m: float = 2.0) -> TimePoseIndex:
    idx = TimePoseIndex(max_dt_s=max_dt_s, max_pose_m=max_pose_m)
    for cap in captures:
        idx.add(cap)
    return idx
