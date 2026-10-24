"""The normalized detection record both platforms converge on before CORRELATE.

This is the single source of truth for what a defect looks like once detected.
It mirrors DATA_INPUTS.md ("Convergence point") exactly. Code fills every field;
the LLM never writes here. The `geometry` block starts null and is filled by the
LiDAR depth-estimation stage (drone side) before severity is computed.
"""
from __future__ import annotations

import time
import uuid
from dataclasses import asdict, dataclass, field
from typing import Optional

# Defect classes, split by platform. Code validates against these; a detector
# that emits anything else is rejected at ingest rather than silently narrated.
# efflorescence and rust_streak are surface stains the camera detector reports
# alongside structural defects (present in real Isaac-Sim bridge bags).
DRONE_CLASSES = ("crack", "spall", "exposed_rebar", "corrosion_stain",
                 "efflorescence", "rust_streak")
# missing_block / undermining / crack are the underwater defects in the
# drone+ROV Craigie bag (run_20261024_134808): a dislodged wall block, a
# washed-out footing, and a crack below the water line seen by sonar.
# sonar_anomaly is what the generic sonar detector reports: "something unusual
# in the sonar return here", with no defect type (the model was not trained on
# structural defects).
ROV_CLASSES = ("scour", "section_loss", "pier_damage", "missing_block", "undermining", "crack",
               "sonar_anomaly")
ALL_CLASSES = DRONE_CLASSES + ROV_CLASSES

PLATFORMS = ("drone", "rov")
CHANGE_STATES = ("new", "existing", "worsening")


@dataclass
class Geodetic:
    lat: Optional[float] = None
    lon: Optional[float] = None
    elevation: Optional[float] = None


@dataclass
class StructureRef:
    """Element-relative location (AASHTO MBEI / SNBI element referencing)."""
    element_id: str = ""
    face: str = ""
    station: Optional[float] = None
    offset: Optional[float] = None
    elevation_or_depth: Optional[float] = None


@dataclass
class Location:
    geodetic: Geodetic = field(default_factory=Geodetic)
    structure: StructureRef = field(default_factory=StructureRef)


@dataclass
class Geometry:
    """Filled by the LiDAR depth-estimation stage. Null at detection time."""
    depth_mm: Optional[float] = None
    area_mm2: Optional[float] = None
    deflection_mm: Optional[float] = None

    def is_measured(self) -> bool:
        return any(v is not None for v in (self.depth_mm, self.area_mm2, self.deflection_mm))


@dataclass
class Pose:
    """6-DoF pose carried with every capture; the key the retrace index uses."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class Detection:
    """The normalized detection record (DATA_INPUTS.md convergence point)."""
    detection_id: str
    mission_id: str
    platform: str
    cls: str
    confidence: float
    timestamp_utc: float               # epoch seconds (ms precision preserved as float)
    pose: Pose = field(default_factory=Pose)
    location: Location = field(default_factory=Location)
    geometry: Geometry = field(default_factory=Geometry)
    evidence_frame_ids: list = field(default_factory=list)
    model_version: str = ""
    change_status: Optional[str] = None       # set by CORRELATE, never by the LLM
    severity: Optional[str] = None            # set by SEVERITY rules, never by the LLM
    severity_reason: Optional[str] = None     # the rules that produced `severity` (code-generated)
    recommended_action: Optional[str] = None  # fixed action text keyed on severity (code-generated)
    source_frame_id: str = ""                 # the frame/scan that produced the detection
    confirmed: Optional[bool] = None          # ROV: camera-confirmed a sonar hit (clarity-gated)
    world_pos: Optional[list] = None          # [x,y,z] world position used for 3D viz / crop

    @staticmethod
    def new_id() -> str:
        return f"D-{uuid.uuid4().hex[:12]}"

    def validate(self) -> None:
        """Code enforces: reject malformed detections at the boundary."""
        if self.platform not in PLATFORMS:
            raise ValueError(f"platform must be one of {PLATFORMS}, got {self.platform!r}")
        allowed = DRONE_CLASSES if self.platform == "drone" else ROV_CLASSES
        if self.cls not in allowed:
            raise ValueError(f"class {self.cls!r} not valid for platform {self.platform!r} (allowed: {allowed})")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(f"confidence must be in [0,1], got {self.confidence}")
        if self.change_status is not None and self.change_status not in CHANGE_STATES:
            raise ValueError(f"change_status must be one of {CHANGE_STATES}")

    def to_dict(self) -> dict:
        d = asdict(self)
        d["class"] = d.pop("cls")   # JSON uses "class" per DATA_INPUTS.md; Python can't
        return d


def make_detection(mission_id: str, platform: str, cls: str, confidence: float,
                   timestamp_utc: Optional[float] = None, **kwargs) -> Detection:
    """Factory that stamps an id and validates. Use this instead of Detection(...) directly."""
    det = Detection(
        detection_id=Detection.new_id(),
        mission_id=mission_id,
        platform=platform,
        cls=cls,
        confidence=float(confidence),
        timestamp_utc=timestamp_utc if timestamp_utc is not None else time.time(),
        **kwargs,
    )
    det.validate()
    return det
