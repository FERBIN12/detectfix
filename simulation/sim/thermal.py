"""Simulated LWIR thermal camera (FLIR Boson 640 class, 640x512).

HONEST APPROXIMATION -- READ THIS
=================================
Isaac Sim has no physics-based thermal (long-wave infrared) sensor. Real surface
temperature depends on emissivity, solar loading history, thermal mass, wind,
evaporative cooling, view angle and the atmospheric path -- none of which Isaac
simulates. This module is therefore a *plausible approximation*, not a radiometric
model: it paints per-class base temperatures onto a semantic-label image, adds a
little stable spatial structure, attenuates toward ambient with range, blurs to
the optics, and adds per-frame NETD noise. It is good enough to make a convincing
ironbow stream and to demonstrate that defects (e.g. subsurface moisture) read
colder than sound masonry -- but the absolute numbers are illustrative only.

Public API
----------
synth(labels, id2class, depth, t, ambient_c=12.0, rng=None)
    -> (temp_c float32 HxW, ironbow uint8 HxWx3 RGB)   resized to 640x512.
ironbow(temp_c, lo, hi) -> uint8 HxWx3 RGB
"""

from __future__ import annotations

import numpy as np
import cv2

# FLIR Boson 640 native resolution.
W_OUT = 640
H_OUT = 512

# NETD (noise-equivalent temperature difference), degrees C, per-frame gaussian.
NETD_C = 0.05

# Per-class temperature offset relative to ambient (degrees C). These are the
# "signatures" the sim paints; see module docstring for the honesty caveat.
CLASS_DT = {
    "sky": -25.0,
    "water": -3.0,
    "granite": 0.0,
    "granite_wet": -2.5,
    "concrete": 1.0,
    "asphalt": 6.0,
    "steel": 2.0,
    "steel_rust": 2.5,
    "brick": 1.5,
    "vegetation": -1.0,
    "vehicle": 15.0,
    "boat": 3.0,
    "drone": 8.0,
    "defect_moisture": -4.0,
    "defect_delamination": 3.5,
    "defect_crack": -1.5,
    "default": 0.0,
}


def _class_dt(name: str) -> float:
    return CLASS_DT.get(name, CLASS_DT["default"])


# --- precomputed 256-entry ironbow LUT --------------------------------------
# Control points of the ironbow colour map (position, R, G, B) in 0..1.
_IRONBOW_STOPS = np.array([
    [0.00, 0.0,  0.0,  0.0],    # black
    [0.18, 0.11, 0.0,  0.42],   # dark blue/indigo
    [0.38, 0.47, 0.0,  0.55],   # purple
    [0.58, 0.86, 0.12, 0.20],   # red
    [0.75, 1.0,  0.45, 0.0],    # orange
    [0.90, 1.0,  0.86, 0.12],   # yellow
    [1.00, 1.0,  1.0,  1.0],    # white
], dtype=np.float32)


def _build_ironbow_lut() -> np.ndarray:
    """256x3 uint8 LUT: ironbow colour for normalised value i/255."""
    xs = _IRONBOW_STOPS[:, 0]
    pos = np.linspace(0.0, 1.0, 256, dtype=np.float32)
    r = np.interp(pos, xs, _IRONBOW_STOPS[:, 1])
    g = np.interp(pos, xs, _IRONBOW_STOPS[:, 2])
    b = np.interp(pos, xs, _IRONBOW_STOPS[:, 3])
    rgb = np.stack([r, g, b], axis=1)
    return (np.clip(rgb, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


_IRONBOW_LUT = _build_ironbow_lut()        # (256, 3) uint8


def ironbow(temp_c: np.ndarray, lo: float, hi: float) -> np.ndarray:
    """Map a temperature image to an 8-bit RGB ironbow image.

    The ironbow palette runs black -> blue -> purple -> red -> orange -> yellow
    -> white as temperature increases. ``lo``/``hi`` set the display range
    (values are clipped). Returns an ``HxWx3`` uint8 RGB array.

    Uses a precomputed 256-entry LUT and ``np.take`` instead of three
    per-pixel ``np.interp`` passes.
    """
    temp_c = np.asarray(temp_c, dtype=np.float32)
    span = max(float(hi) - float(lo), 1e-6)
    norm = np.clip((temp_c - lo) * (1.0 / span), 0.0, 1.0)
    idx = (norm * 255.0 + 0.5).astype(np.uint8)
    # LUT lookup -> (..., 3) uint8, identical to the old interp-then-quantise.
    return _IRONBOW_LUT[idx]


def _stable_class_noise(labels: np.ndarray, id2class: dict, amp: float = 0.6) -> np.ndarray:
    """Smooth low-frequency spatial variation in degrees C, in [-amp, amp].

    Built from a small random field upsampled to image size, seeded per the set
    of class ids present so it is stable frame-to-frame (does not flicker).
    """
    h, w = labels.shape
    # Seed from the sorted unique ids so the field is deterministic per scene.
    uids = np.unique(labels)
    seed = int((uids.astype(np.int64) * 2654435761 % (2**31)).sum()) & 0x7FFFFFFF
    rng = np.random.default_rng(seed)
    coarse = rng.standard_normal((8, 8)).astype(np.float32)
    field = cv2.resize(coarse, (w, h), interpolation=cv2.INTER_CUBIC)
    # normalise to +-amp
    m = np.max(np.abs(field))
    if m > 1e-6:
        field = field / m * amp
    return field.astype(np.float32)


# --- cached low-frequency noise field at output resolution ------------------
# Keyed by the sorted tuple of unique ids present; the field only depends on
# that set (deterministic seed), so we regenerate it only when the scene's id
# set changes rather than every frame.
_NOISE_CACHE: dict = {}
_NOISE_CACHE_MAX = 32


def _stable_class_noise_cached(uids: np.ndarray, amp: float = 0.6) -> np.ndarray:
    """Cached variant of :func:`_stable_class_noise` at output resolution.

    Produces the identical field to ``_stable_class_noise(labels_r, ...)`` for
    an output-sized ``labels_r`` whose unique ids are ``uids`` -- the field only
    depends on the id set, not pixel layout.
    """
    key = (tuple(int(u) for u in uids), float(amp))
    cached = _NOISE_CACHE.get(key)
    if cached is not None:
        return cached
    seed = int((uids.astype(np.int64) * 2654435761 % (2**31)).sum()) & 0x7FFFFFFF
    rng = np.random.default_rng(seed)
    coarse = rng.standard_normal((8, 8)).astype(np.float32)
    field = cv2.resize(coarse, (W_OUT, H_OUT), interpolation=cv2.INTER_CUBIC)
    m = np.max(np.abs(field))
    if m > 1e-6:
        field = field / m * amp
    field = field.astype(np.float32)
    if len(_NOISE_CACHE) >= _NOISE_CACHE_MAX:
        _NOISE_CACHE.clear()
    _NOISE_CACHE[key] = field
    return field


def synth(labels, id2class, depth, t, ambient_c: float = 12.0, rng=None):
    """Synthesise a thermal frame from a semantic-label image.

    Parameters
    ----------
    labels : HxW int array
        Semantic id per pixel.
    id2class : dict[int, str]
        Maps semantic id -> class name. Unknown / unlabelled ids read as "sky".
    depth : HxW float array
        Per-pixel range from the camera, metres. Used for atmospheric
        attenuation toward ambient.
    t : float
        Time (seconds). Reserved for temporal effects; NETD noise is per-call.
    ambient_c : float
        Ambient air temperature, degrees C.
    rng : np.random.Generator or None
        Source for the per-frame NETD noise. Defaults to a fresh default_rng.

    Returns
    -------
    (temp_c, ironbow_rgb)
        temp_c      : float32 HxW temperature image (resized to 640x512).
        ironbow_rgb : uint8   HxWx3 RGB ironbow image (640x512), auto-ranged.
    """
    if rng is None:
        rng = np.random.default_rng()

    labels = np.asarray(labels)
    if labels.ndim != 2:
        raise ValueError("labels must be a 2D HxW integer image")

    # --- resize label & depth to output BEFORE colouring (labels: NEAREST) ---
    labels_r = cv2.resize(
        labels.astype(np.int32), (W_OUT, H_OUT), interpolation=cv2.INTER_NEAREST
    )
    depth = np.asarray(depth, dtype=np.float32)
    if depth.shape != (H_OUT, W_OUT):
        depth_r = cv2.resize(depth, (W_OUT, H_OUT), interpolation=cv2.INTER_NEAREST)
    else:
        depth_r = depth

    # --- per-class base temperature via a dense LUT (np.take) -----------------
    # uids (sorted unique ids present) seeds the stable-noise field below, and
    # we build a dense id -> dT table so the per-pixel mapping is a single
    # gather (``take``) instead of a searchsorted + gather over the whole image.
    uids = np.unique(labels_r)
    max_id = int(uids[-1]) if uids.shape[0] else 0
    min_id = int(uids[0]) if uids.shape[0] else 0
    if 0 <= min_id and max_id <= 65535:
        # Compact id range: direct dense lookup table (fast path).
        lut = np.empty(max_id + 1, dtype=np.float32)
        for u in uids:
            lut[int(u)] = _class_dt(id2class.get(int(u), "sky"))
        dt_field = np.take(lut, labels_r)
    else:
        # Sparse / negative ids: map via searchsorted into a per-uid table.
        dt_vals = np.array(
            [_class_dt(id2class.get(int(u), "sky")) for u in uids],
            dtype=np.float32,
        )
        dt_field = dt_vals[np.searchsorted(uids, labels_r)]
    temp = ambient_c + dt_field

    # --- stable low-frequency spatial variation (+-0.6 C), cached -------------
    temp = temp + _stable_class_noise_cached(uids, amp=0.6)

    # --- atmospheric attenuation toward ambient with range --------------------
    # tau ~ exp(-depth/400): far surfaces pull toward ambient.
    tau = np.exp(np.clip(depth_r, 0.0, None) * (-1.0 / 400.0)).astype(np.float32)
    temp = ambient_c + (temp - ambient_c) * tau

    # --- optical blur (sigma ~1 px) -------------------------------------------
    temp = cv2.GaussianBlur(temp, (0, 0), sigmaX=1.0, sigmaY=1.0)

    # --- temporal NETD noise (~0.05 C) ----------------------------------------
    temp = temp + rng.normal(0.0, NETD_C, size=temp.shape).astype(np.float32)
    temp = temp.astype(np.float32)

    # --- auto-range (2nd/98th pct, min span 6 C) and colour -------------------
    lo, hi = np.percentile(temp, (2.0, 98.0))
    lo = float(lo)
    hi = float(hi)
    if hi - lo < 6.0:
        mid = 0.5 * (lo + hi)
        lo, hi = mid - 3.0, mid + 3.0
    rgb = ironbow(temp, lo, hi)

    return temp, rgb
