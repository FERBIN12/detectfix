"""DETECT (pipeline step 3) + the camera-confirm half of RETRACE (step 4).

Two detector slots, each behind one interface so a mock runs offline and a live
one loads real weights -- the same mock/live-adapter pattern the dashboard uses:

  drone RGB   -> CameraDetector   (crack-detection-yolov8, YOLOv8-seg)
  rov  sonar  -> SonarDetector    (KD-YOLOX-ViT)

The detectors only find and classify. Location refinement, geometry, change
status and severity are added by later deterministic stages -- code enforces.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from .indexing import Capture, TimePoseIndex
from .schema import Detection, make_detection


# Map the real model's class names to the pipeline's drone schema classes.
# Default is for "Kapuni Model A" (iloveass/kompon-damage-severity-detection).
# Unmapped model classes are dropped rather than mis-narrated.
CAMERA_CLASS_MAP = {
    "structural_crack": "crack",
    "crack": "crack",
    "microcrack": "crack",
    "spalling": "spall",
    "spall": "spall",
    "rebar_corrosion": "corrosion_stain",
    "corrosion": "corrosion_stain",
    "exposed_rebar": "exposed_rebar",
    "severe_distress": "spall",   # severe distress reads as structural loss
    "efflorescence": "efflorescence",
    "rust_streak": "rust_streak",
}


class CameraDetector:
    """Slot: real multi-class concrete-defect YOLOv8-seg ("Kapuni Model A",
    iloveass/kompon-damage-severity-detection on the Hugging Face Hub).

    Live: loads the Ultralytics checkpoint and segments the drone's RGB frames,
    mapping the model's class names onto the pipeline's drone classes
    (crack / spall / corrosion_stain / exposed_rebar). Mock: replays detections
    carried in the capture payload so the flow runs with nothing installed.
    """

    model_version = "camera-seg@mock"

    def __init__(self, weights: Optional[str] = None, conf_threshold: float = 0.25,
                 class_map: Optional[dict] = None):
        self.weights = weights
        self.conf_threshold = conf_threshold
        self.class_map = class_map or CAMERA_CLASS_MAP
        self._model = None  # lazily loaded ultralytics model in live mode

    def _load(self):
        if self._model is None and self.weights:
            from ultralytics import YOLO  # type: ignore
            self._model = YOLO(self.weights)
            self.model_version = f"camera-seg@{Path(self.weights).stem}"
        return self._model

    def detect(self, mission_id: str, cap: Capture) -> list[Detection]:
        model = self._load()
        if model is None:
            return self._mock_detect(mission_id, cap)
        return self._live_detect(mission_id, cap, model)

    def _mock_detect(self, mission_id: str, cap: Capture) -> list[Detection]:
        out = []
        for d in cap.payload.get("detections", []):
            if float(d.get("confidence", 0)) < self.conf_threshold:
                continue
            out.append(make_detection(
                mission_id=mission_id, platform="drone", cls=d["class"],
                confidence=float(d["confidence"]), timestamp_utc=cap.timestamp_utc,
                pose=cap.pose, source_frame_id=cap.capture_id,
                evidence_frame_ids=[cap.capture_id], model_version=self.model_version,
            ))
        return out

    def _live_detect(self, mission_id: str, cap: Capture, model) -> list[Detection]:
        img = cap.payload.get("image_path") or cap.payload.get("image")
        if img is None:
            return []
        results = model.predict(img, conf=self.conf_threshold, verbose=False)
        out = []
        names = getattr(model, "names", {})
        for r in results:
            for i, box in enumerate(getattr(r, "boxes", []) or []):
                raw = names.get(int(box.cls), str(int(box.cls)))
                cls_name = self.class_map.get(raw)
                if cls_name is None:
                    continue  # model class not in the drone schema -> drop, don't guess
                det = make_detection(
                    mission_id=mission_id, platform="drone", cls=cls_name,
                    confidence=float(box.conf), timestamp_utc=cap.timestamp_utc,
                    pose=cap.pose, source_frame_id=cap.capture_id,
                    evidence_frame_ids=[cap.capture_id], model_version=self.model_version,
                )
                det._raw_class = raw                          # type: ignore[attr-defined]
                det._bbox = _box_xyxy(box)                    # type: ignore[attr-defined]
                det._mask_px = _mask_area_px(r, i)            # type: ignore[attr-defined]
                out.append(det)
        return out


class SonarDetector:
    """ROV sonar detector slot.

    Live (.pt weights): a generic sonar object detector run through
    Ultralytics -- by default "Sonnet Model C" (weights.resolve_sonar_weights),
    trained on side-scan seabed hazards, not structural defects. Every box it
    returns becomes class "sonar_anomaly"; code then filters boxes, places them
    in the world from the fan geometry, and clusters repeats (pipeline.rov).
    Other weights: the KD-YOLOX-ViT predictor hook. Mock: replays payload
    detections.
    """

    model_version = "sonar@mock"

    def __init__(self, weights: Optional[str] = None, conf_threshold: float = 0.3):
        self.weights = weights
        self.conf_threshold = conf_threshold
        self._predictor = None

    def _load(self):
        if self._predictor is None and self.weights:
            if str(self.weights).endswith(".pt"):
                from ultralytics import YOLO  # type: ignore
                model = YOLO(self.weights)

                def predict(img, conf):
                    out = []
                    for r in model.predict(img, conf=conf, verbose=False):
                        names = getattr(model, "names", {})
                        for b in getattr(r, "boxes", []) or []:
                            out.append({"class": "sonar_anomaly",
                                        "raw_class": names.get(int(b.cls), str(int(b.cls))),
                                        "confidence": float(b.conf),
                                        "box": [float(v) for v in b.xyxy[0].tolist()]})
                    return out

                self._predictor = predict
                self.model_version = f"sonar-yolo@{Path(self.weights).stem}"
            else:
                # KD-YOLOX-ViT is YOLOX-based; its predictor is project-specific.
                from yolox_vit import build_predictor  # type: ignore
                self._predictor = build_predictor(self.weights)
                self.model_version = f"kd-yolox-vit@{Path(self.weights).stem}"
        return self._predictor

    def predict_image(self, img) -> list[dict]:
        """Raw model output for one sonar image (ndarray or path): dicts with
        class, raw_class, confidence, box [x1,y1,x2,y2]. [] without weights."""
        predictor = self._load()
        return predictor(img, conf=self.conf_threshold) if predictor else []

    def detect(self, mission_id: str, cap: Capture) -> list[Detection]:
        predictor = self._load()
        if predictor is None:
            return self._mock_detect(mission_id, cap)
        return self._live_detect(mission_id, cap, predictor)

    def _mock_detect(self, mission_id: str, cap: Capture) -> list[Detection]:
        out = []
        for d in cap.payload.get("detections", []):
            if float(d.get("confidence", 0)) < self.conf_threshold:
                continue
            out.append(make_detection(
                mission_id=mission_id, platform="rov", cls=d["class"],
                confidence=float(d["confidence"]), timestamp_utc=cap.timestamp_utc,
                pose=cap.pose, source_frame_id=cap.capture_id,
                evidence_frame_ids=[cap.capture_id], model_version=self.model_version,
            ))
        return out

    def _live_detect(self, mission_id: str, cap: Capture, predictor) -> list[Detection]:
        img = cap.payload.get("image_path") or cap.payload.get("image")
        if img is None:
            return []
        preds = predictor(img, conf=self.conf_threshold)
        out = []
        for p in preds:
            det = make_detection(
                mission_id=mission_id, platform="rov", cls=p["class"],
                confidence=float(p["confidence"]), timestamp_utc=cap.timestamp_utc,
                pose=cap.pose, source_frame_id=cap.capture_id,
                evidence_frame_ids=[cap.capture_id], model_version=self.model_version,
            )
            det._raw_class = p.get("raw_class")   # type: ignore[attr-defined]
            det._bbox = p.get("box")              # type: ignore[attr-defined]
            out.append(det)
        return out


def run_detection(mission_id: str, index: TimePoseIndex, captures: list[Capture],
                  camera: CameraDetector, sonar: SonarDetector) -> list[Detection]:
    """Step 3: run both detectors over their primary streams."""
    detections: list[Detection] = []
    for cap in captures:
        if cap.kind == "rgb":
            detections.extend(camera.detect(mission_id, cap))
        elif cap.kind == "sonar":
            detections.extend(sonar.detect(mission_id, cap))
    return detections


def confirm_rov_detections(index: TimePoseIndex, detections: list[Detection]) -> None:
    """RETRACE (camera-confirm half, step 4): for each sonar defect, pull the
    recorded ROV camera frame co-located with it and mark confirmed.

    Clarity-gated: if no camera frame falls inside the time/pose gate (turbid
    stretch), the defect stays sonar-only (confirmed=False) -- expected, not an
    error.
    """
    for det in detections:
        if det.platform != "rov":
            continue
        frame = index.retrace("rov_camera", det.timestamp_utc, det.pose)
        if frame is None:
            det.confirmed = False
            continue
        det.confirmed = True
        if frame.capture_id not in det.evidence_frame_ids:
            det.evidence_frame_ids.append(frame.capture_id)


def _box_xyxy(box):
    try:
        xy = box.xyxy[0].tolist()
        return [float(v) for v in xy]
    except Exception:
        return None


def _mask_area_px(result, i):
    """Pixel area of the i-th instance mask, if the model produced masks."""
    try:
        masks = getattr(result, "masks", None)
        if masks is None:
            return None
        data = masks.data[i]
        return float(data.sum().item())
    except Exception:
        return None
