"""Download + resolve model weights.

The drone camera detector uses a real multi-class concrete-defect YOLOv8-seg
checkpoint ("Kapuni Model A", iloveass/kompon-damage-severity-detection on the
Hugging Face Hub): classes crack / spalling / rebar corrosion / severe distress
/ microcrack. The file is best.pt (~24 MB), a genuine ultralytics
SegmentationModel.

Weights land in bridge-inspection/weights/ (gitignored). Resolution order:
  1. an explicit path passed in / in config,
  2. a previously downloaded copy in weights/,
  3. download from the Hub.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent.parent
WEIGHTS_DIR = HERE / "weights"

# The drone defect segmentation model.
CAMERA_REPO = "iloveass/kompon-damage-severity-detection"
CAMERA_FILE = "best.pt"
CAMERA_LOCAL = WEIGHTS_DIR / "kapuni_model_a_seg.pt"


def resolve_camera_weights(explicit: Optional[str] = None, download: bool = True) -> Optional[str]:
    """Return a local path to the camera defect weights, downloading if needed."""
    if explicit:
        p = Path(explicit)
        if p.exists():
            return str(p)
    if CAMERA_LOCAL.exists():
        return str(CAMERA_LOCAL)
    if not download:
        return None
    return download_camera_weights()


def download_camera_weights() -> str:
    """Fetch best.pt from the Hub into weights/ and return its path."""
    from huggingface_hub import hf_hub_download
    WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
    cached = hf_hub_download(repo_id=CAMERA_REPO, filename=CAMERA_FILE)
    # Copy out of the HF cache into our weights/ dir under a clear name.
    import shutil
    shutil.copyfile(cached, CAMERA_LOCAL)
    return str(CAMERA_LOCAL)
