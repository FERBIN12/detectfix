"""Small mesh helpers (numpy -> USD): mesh, superellipsoid, cylinder, box."""
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt


def _spow(x, p):
    return np.sign(x) * np.abs(x) ** p


def superellipsoid(cx, cy, cz, ax, ay, az, e_v=0.35, e_h=0.35, nu=48, nv=24,
                   z_clip=None):
    """Rounded box. e -> 0 is boxy, 1 is ellipsoid. Returns (points, counts, indices).
    z_clip=(zmin,zmax) flattens points outside that slab (cheap way to make shells/caps)."""
    u = np.linspace(-np.pi, np.pi, nu, endpoint=False)
    v = np.linspace(-np.pi / 2, np.pi / 2, nv)
    pts = []
    for vv in v:
        cv, sv = np.cos(vv), np.sin(vv)
        for uu in u:
            x = ax * _spow(cv, e_v) * _spow(np.cos(uu), e_h)
            y = ay * _spow(cv, e_v) * _spow(np.sin(uu), e_h)
            z = az * _spow(sv, e_v)
            pts.append((cx + x, cy + y, cz + z))
    pts = np.array(pts)
    if z_clip is not None:
        pts[:, 2] = np.clip(pts[:, 2], z_clip[0], z_clip[1])
    counts, idx = [], []
    for j in range(nv - 1):
        for i in range(nu):
            a = j * nu + i
            b = j * nu + (i + 1) % nu
            c = (j + 1) * nu + (i + 1) % nu
            d = (j + 1) * nu + i
            counts.append(4)
            idx += [a, b, c, d]
    return pts, counts, idx


def add_mesh(stage, path, geo, mat):
    pts, counts, idx = geo
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    # smooth shading: per-vertex normals averaged from the faces around each vertex
    P = np.asarray(pts, float); nrm = np.zeros_like(P); ii = np.asarray(idx).reshape(-1, 4)
    fn = np.cross(P[ii[:, 2]] - P[ii[:, 0]], P[ii[:, 3]] - P[ii[:, 1]])
    for k in range(4):
        np.add.at(nrm, ii[:, k], fn)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True); nrm = np.where(ln > 1e-12, nrm / np.maximum(ln, 1e-12), [0, 0, 1])
    m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in nrm]))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)
    return m


def box(stage, path, center, size, mat):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI(c.GetPrim()).Bind(mat)
    return c


def cyl(stage, path, center, radius, height, axis, mat):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius); c.CreateHeightAttr(height); c.CreateAxisAttr(axis)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*center))
    UsdShade.MaterialBindingAPI(c.GetPrim()).Bind(mat)
    return c

