"""Tests for sim/thermal.py (the simulated LWIR camera).

Covered here (numpy + cv2 only, no ROS needed):

  * synth() output shapes/dtypes: temp_c is float32 640x512, ironbow is
    uint8 640x512x3;
  * defect classes read COLDER than granite in temp_c (moisture / crack);
  * ironbow() brightness is monotone non-decreasing with temperature.

The module is imported by path so the test does not depend on sys.path layout.
"""
import os
import importlib.util

import numpy as np
import pytest

# ---- import sim/thermal.py by path -----------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_PATH = os.path.join(_HERE, "..", "sim", "thermal.py")
_spec = importlib.util.spec_from_file_location("thermal_under_test", _PATH)
thermal = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(thermal)


def _scene():
    """Small labelled scene: columns of granite / water / defect_moisture /
    defect_crack, plus an id2class map and a flat depth field."""
    h, w = 64, 80
    labels = np.zeros((h, w), dtype=np.int32)
    # four vertical bands
    labels[:, 0:20] = 1    # granite
    labels[:, 20:40] = 2   # water
    labels[:, 40:60] = 3   # defect_moisture
    labels[:, 60:80] = 4   # defect_crack
    id2class = {
        1: "granite",
        2: "water",
        3: "defect_moisture",
        4: "defect_crack",
    }
    depth = np.full((h, w), 5.0, dtype=np.float32)
    return labels, id2class, depth


def test_synth_shapes_and_dtypes():
    labels, id2class, depth = _scene()
    rng = np.random.default_rng(0)
    temp, rgb = thermal.synth(labels, id2class, depth, t=0.0, ambient_c=12.0, rng=rng)

    assert temp.shape == (thermal.H_OUT, thermal.W_OUT) == (512, 640)
    assert temp.dtype == np.float32
    assert rgb.shape == (thermal.H_OUT, thermal.W_OUT, 3)
    assert rgb.dtype == np.uint8


def test_defects_colder_than_granite():
    """Defect regions (moisture, crack) must be colder than granite in temp_c.

    We disable NETD noise influence by using a fixed rng and comparing region
    medians, which are robust to the ~0.05 C per-pixel noise.
    """
    labels, id2class, depth = _scene()
    rng = np.random.default_rng(42)
    temp, _ = thermal.synth(labels, id2class, depth, t=0.0, ambient_c=12.0, rng=rng)

    # resized label map to locate regions in the 640x512 output
    import cv2
    labels_r = cv2.resize(labels, (thermal.W_OUT, thermal.H_OUT),
                          interpolation=cv2.INTER_NEAREST)

    granite_med = np.median(temp[labels_r == 1])
    moisture_med = np.median(temp[labels_r == 3])
    crack_med = np.median(temp[labels_r == 4])

    assert moisture_med < granite_med, (
        f"moisture ({moisture_med:.2f}) not colder than granite ({granite_med:.2f})")
    assert crack_med < granite_med, (
        f"crack ({crack_med:.2f}) not colder than granite ({granite_med:.2f})")


def test_ironbow_monotone_brightness():
    """Perceived brightness of the ironbow palette is non-decreasing with
    temperature across the display range."""
    lo, hi = 0.0, 30.0
    ramp = np.linspace(lo, hi, 256, dtype=np.float32).reshape(1, -1)
    rgb = thermal.ironbow(ramp, lo, hi).astype(np.float32)[0]  # (256,3)

    # luminance (Rec. 601)
    lum = 0.299 * rgb[:, 0] + 0.587 * rgb[:, 1] + 0.114 * rgb[:, 2]
    diffs = np.diff(lum)
    # Allow tiny non-monotonic dips from 8-bit quantisation; require the trend
    # to be clearly increasing and no large backward steps.
    assert diffs.min() >= -1.0, f"brightness drops by {diffs.min():.2f} (>1 LSB)"
    assert lum[-1] > lum[0] + 100.0, "palette does not brighten across the range"
    # overwhelmingly non-decreasing
    assert np.mean(diffs >= 0) > 0.95


def test_ironbow_endpoints():
    """Ironbow maps the low end to near-black and the high end to near-white."""
    lo, hi = 10.0, 20.0
    low = thermal.ironbow(np.array([[lo]], dtype=np.float32), lo, hi)[0, 0]
    high = thermal.ironbow(np.array([[hi]], dtype=np.float32), lo, hi)[0, 0]
    assert int(low.sum()) < 30, f"low end not black: {low}"
    assert int(high.min()) > 220, f"high end not white: {high}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
