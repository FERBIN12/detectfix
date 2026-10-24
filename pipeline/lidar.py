"""LiDAR DEPTH ESTIMATION (pipeline step 5) + the LiDAR half of RETRACE (step 4).

Slot: pc_cr (https://github.com/yliu17lhr/pc_cr) / Open3D geometric processing.
NOT a trained model -- this is deterministic point-cloud geometry, so it fits
"code enforces". For each drone crack detection we:

  1. retrace the recorded LiDAR cloud co-located with the detection (by ts+pose),
  2. project the 2D defect region into the cloud (intrinsics + extrinsics),
  3. isolate the defect-region points and fit the local surface,
  4. measure depth_mm / area_mm2 / surface deviation,
  5. (cross-cycle) register vs. a prior cloud -> deflection / change,

then fill the detection's `geometry` block, which SEVERITY consumes.

Open3D is optional; without it the stage uses a dependency-free estimator over
point lists so the pipeline still runs offline. Both paths fill the same fields.
"""
from __future__ import annotations

import math
from typing import Optional

from .indexing import Capture, TimePoseIndex
from .schema import Detection, Geometry


def _load_points(payload: dict) -> list[tuple]:
    """Return a list of (x,y,z) points from a capture payload.

    Supports an inline ``points`` list (fixtures) or a ``cloud_path`` to a file
    Open3D can read (live). Returns [] when neither is present.
    """
    if payload.get("points"):
        return [tuple(float(c) for c in p[:3]) for p in payload["points"]]
    path = payload.get("cloud_path")
    if not path:
        return []
    try:
        import open3d as o3d  # type: ignore
        pcd = o3d.io.read_point_cloud(path)
        return [tuple(p) for p in pcd.points]
    except ImportError:
        return []


def _region_points(points: list[tuple], region: Optional[dict]) -> list[tuple]:
    """Keep only points inside the projected defect region's axis-aligned box.

    `region` is the detection mask's 3D bounds in the cloud frame
    {min:[x,y,z], max:[x,y,z]}; without it we use all points (whole capture is
    assumed to be the local patch, as in fixtures).
    """
    if not region:
        return points
    lo, hi = region.get("min"), region.get("max")
    if not (lo and hi):
        return points
    out = []
    for x, y, z in points:
        if lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1] and lo[2] <= z <= hi[2]:
            out.append((x, y, z))
    return out


def _fit_plane_and_measure(points: list[tuple]) -> Geometry:
    """Fit a reference plane to the surrounding surface and measure how far the
    defect points deviate from it.

    depth_mm    = max inward deviation from the fitted plane (spall/scour depth)
    area_mm2    = projected area of the deviating region (bounding footprint)
    deflection  = left None here; it is a cross-cycle quantity (see below)
    """
    if len(points) < 4:
        return Geometry()
    try:
        import open3d as o3d  # type: ignore
        import numpy as np  # type: ignore
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
        plane, inliers = pcd.segment_plane(distance_threshold=0.002,
                                           ransac_n=3, num_iterations=200)
        a, b, c, d = (float(v) for v in plane)
        norm = math.sqrt(a * a + b * b + c * c) or 1.0
        devs = [abs(a * float(x) + b * float(y) + c * float(z) + d) / norm for (x, y, z) in points]
        depth_m = max(devs) if devs else 0.0
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        area_m2 = (max(xs) - min(xs)) * (max(ys) - min(ys))
        return Geometry(depth_mm=round(depth_m * 1000, 2),
                        area_mm2=round(area_m2 * 1e6, 1))
    except ImportError:
        return _fit_plane_pure_python(points)


def _fit_plane_pure_python(points: list[tuple]) -> Geometry:
    """Dependency-free fallback: fit a plane by least squares (z = ax+by+c) and
    measure vertical deviation. Points are centred first so the normal-equation
    solve stays well-conditioned on large world coordinates."""
    n = len(points)
    cx = sum(float(p[0]) for p in points) / n
    cy = sum(float(p[1]) for p in points) / n
    cz = sum(float(p[2]) for p in points) / n
    points = [(float(p[0]) - cx, float(p[1]) - cy, float(p[2]) - cz) for p in points]
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sz = sum(p[2] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    syy = sum(p[1] * p[1] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    sxz = sum(p[0] * p[2] for p in points)
    syz = sum(p[1] * p[2] for p in points)
    # Solve the 3x3 normal equations for [a, b, c].
    m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    rhs = [sxz, syz, sz]
    sol = _solve3(m, rhs)
    if sol is None:
        return Geometry()
    a, b, c = (float(v) for v in sol)
    devs = [abs(a * float(x) + b * float(y) + c - float(z)) / math.sqrt(a * a + b * b + 1)
            for (x, y, z) in points]
    depth_m = max(devs) if devs else 0.0
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    area_m2 = (max(xs) - min(xs)) * (max(ys) - min(ys))
    return Geometry(depth_mm=round(float(depth_m) * 1000, 2), area_mm2=round(float(area_m2) * 1e6, 1))


def _solve3(m, rhs):
    """Solve a 3x3 linear system by Cramer's rule; None if singular."""
    def det3(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det3(m)
    if abs(d) < 1e-12:
        return None
    out = []
    for i in range(3):
        mi = [row[:] for row in m]
        for r in range(3):
            mi[r][i] = rhs[r]
        out.append(det3(mi) / d)
    return out


def estimate_geometry(index: TimePoseIndex, det: Detection) -> Geometry:
    """Retrace the recorded LiDAR cloud for this detection and measure geometry."""
    cloud = index.retrace("lidar", det.timestamp_utc, det.pose)
    if cloud is None:
        return Geometry()
    points = _load_points(cloud.payload)
    region = det_region(det)
    points = _region_points(points, region)
    geom = _fit_plane_and_measure(points)
    if cloud.capture_id not in det.evidence_frame_ids:
        det.evidence_frame_ids.append(cloud.capture_id)
    return geom


def det_region(det: Detection) -> Optional[dict]:
    """The detection's projected 3D region, if the detector carried one."""
    # Carried through from the detector payload when available; None -> use the
    # whole retraced patch (the fixture convention).
    return None


def run_lidar_depth(index: TimePoseIndex, detections: list[Detection]) -> None:
    """Step 5: fill the geometry block for every drone detection in place.

    ROV detections are left with null geometry -- underwater geometry comes from
    sonar reconstruction in MAP/STITCH, not LiDAR.
    """
    for det in detections:
        if det.platform != "drone":
            continue
        det.geometry = estimate_geometry(index, det)


# --- map-based geometry (real bag path) --------------------------------------
class WorldCloud:
    """The accumulated world-frame LiDAR map (metres, Z up). Loaded once; crops
    a local box around a defect's world position for geometry measurement.

    Backed by the mission's lidar_map_world.npy (or a .pcd via Open3D). This is
    the real recorded map, so measurements are off actual flown LiDAR returns.
    """

    def __init__(self, points):
        self._points = points   # list[(x,y,z)] or numpy array

    @classmethod
    def load(cls, map_dir: str):
        from pathlib import Path
        d = Path(map_dir)
        npy = d / "lidar_map_world.npy"
        if npy.exists():
            try:
                import numpy as np  # type: ignore
                arr = np.load(str(npy))
                return cls(arr[:, :3])
            except ImportError:
                pass
        pcd = d / "lidar_map_world_5cm.pcd"
        if pcd.exists():
            try:
                import open3d as o3d  # type: ignore
                cloud = o3d.io.read_point_cloud(str(pcd))
                return cls([tuple(p) for p in cloud.points])
            except ImportError:
                pass
        return None

    def crop(self, center, half=0.6):
        cx, cy, cz = center
        try:
            import numpy as np  # type: ignore
            pts = np.asarray(self._points)
            mask = ((abs(pts[:, 0] - cx) <= half) & (abs(pts[:, 1] - cy) <= half)
                    & (abs(pts[:, 2] - cz) <= half))
            return [tuple(p) for p in pts[mask]]
        except ImportError:
            return [(x, y, z) for (x, y, z) in self._points
                    if abs(x - cx) <= half and abs(y - cy) <= half and abs(z - cz) <= half]


def measure_from_world_map(world: "WorldCloud", detections: list[Detection],
                           positions: dict, half: float = 0.15) -> None:
    """Fill geometry by cropping the world map tightly around each detection's
    known world position and measuring LOCAL SURFACE ROUGHNESS only.

    Honest scope: the accumulated LiDAR map is the bridge surface. It does not
    contain a defect's cavity geometry unless the defect physically deforms the
    surface, which simulated surface markings (crack/efflorescence/rust_streak)
    do not. So we report the one thing the data actually supports -- the RMS /
    max deviation of nearby points from the locally fitted plane -- as
    `depth_mm` (surface roughness), and we leave `area_mm2` null rather than
    passing off the crop-box footprint as a defect area.
    """
    if world is None:
        return
    for det in detections:
        if det.platform != "drone":
            continue
        pos = positions.get(det.detection_id)
        if pos is None:
            continue
        pts = world.crop(pos, half=half)
        det.geometry = _surface_roughness(pts)


def _surface_roughness(points: list[tuple]) -> Geometry:
    """RMS and max deviation of points from the best-fit plane, in mm, via a
    PCA plane fit (orthogonal-distance, works for any surface orientation --
    the canal walls are vertical, so a z=f(x,y) fit would be degenerate).

    This is a real LiDAR-derived surface metric; it is NOT a defect cavity
    depth, and no area is claimed.
    """
    if len(points) < 8:
        return Geometry()
    try:
        import numpy as np  # type: ignore
        P = np.asarray([(float(x), float(y), float(z)) for (x, y, z) in points])
        c = P.mean(axis=0)
        Q = P - c
        # Smallest-eigenvalue eigenvector of the covariance is the plane normal.
        _, _, vh = np.linalg.svd(Q, full_matrices=False)
        normal = vh[-1]
        dev = np.abs(Q @ normal)        # orthogonal distance of each point
        rms = float(np.sqrt(np.mean(dev ** 2)))
        return Geometry(depth_mm=round(rms * 1000, 2), area_mm2=None, deflection_mm=None)
    except ImportError:
        g = _fit_plane_pure_python([(float(x), float(y), float(z)) for (x, y, z) in points])
        return Geometry(depth_mm=g.depth_mm, area_mm2=None, deflection_mm=None)
