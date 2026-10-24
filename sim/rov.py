"""BlueROV2 Heavy in Isaac: Blue Robotics' own geometry (brov2h mesh), Lumen lights, a forward multibeam imaging sonar
(Blue View / Oculus M1200d class, 130 x 20 deg, 256 beams) and the low-light camera with an underwater optics model.

    from rov import build_rov, Sonar, murk
    op, cam_path, son_path = build_rov(st, "/World/ROV", PR)

Local frame: +x forward, +y left, +z up (same as base_link in ROS).
"""
import os, math
import numpy as np
from pxr import UsdGeom, UsdShade, UsdLux, Gf, Vt

MESH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "rov", "brov2h.npz")
CEN = np.array([-0.0095, -0.0005, 0.0125])


def build_rov(st, root, PR):
    UsdGeom.Xform.Define(st, root); op = UsdGeom.Xformable(st.GetPrimAtPath(root)).AddTransformOp()
    M = dict(blue=PR.omnipbr(st, root + "/Looks/Blue", (0.02, 0.18, 0.62), 0.35),
             acrylic=PR.omnipbr(st, root + "/Looks/Acrylic", (0.75, 0.80, 0.85), 0.05, 0.1),
             orange=PR.omnipbr(st, root + "/Looks/Orange", (0.85, 0.30, 0.04), 0.45),
             frame=PR.omnipbr(st, root + "/Looks/Frame", (0.03, 0.03, 0.035), 0.5),
             lamp=PR.omnipbr(st, root + "/Looks/Lamp", (1, 1, 1), 0.2, emissive=((1.0, 0.96, 0.88), 40000.0)))
    d = np.load(MESH)
    hull = UsdGeom.Xform.Define(st, root + "/Hull")
    for k, col in enumerate(d["cols"]):
        P = d[f"P{k}"].astype(np.float64) * 0.025 - CEN; n = len(P) // 3
        m = UsdGeom.Mesh.Define(st, f"{root}/Hull/G{k}")
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
        m.CreateFaceVertexCountsAttr([3] * n); m.CreateFaceVertexIndicesAttr(list(range(len(P))))
        c = np.asarray(col, float)
        mm = M["blue"] if c[2] > c[0] + 0.15 else (M["acrylic"] if c.mean() > 0.6 else (M["orange"] if c[0] > c[2] + 0.3 else M["frame"]))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mm)
    for i, sy in enumerate((-0.12, 0.12)):                                   # two Lumen lights, forward
        lens = UsdGeom.Cylinder.Define(st, f"{root}/LumenLens{i}"); lens.CreateRadiusAttr(0.018); lens.CreateHeightAttr(0.004); lens.CreateAxisAttr("X")
        UsdGeom.Xformable(lens).AddTranslateOp().Set(Gf.Vec3d(0.235, sy, 0.10)); UsdShade.MaterialBindingAPI.Apply(lens.GetPrim()).Bind(M["lamp"])
        L = UsdLux.SphereLight.Define(st, f"{root}/Lumen{i}"); L.CreateRadiusAttr(0.015); L.CreateIntensityAttr(2.5e6)
        L.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85)); UsdLux.ShapingAPI.Apply(L.GetPrim()).CreateShapingConeAngleAttr(40.0)
        UsdLux.ShapingAPI(L.GetPrim()).CreateShapingConeSoftnessAttr(0.4)
        lx = UsdGeom.Xformable(L); lx.AddTranslateOp().Set(Gf.Vec3d(0.25, sy, 0.10)); lx.AddRotateYOp().Set(-90.0)
    cam = UsdGeom.Camera.Define(st, root + "/Camera")                         # low-light camera behind the dome, tilted 15 deg down
    cam.CreateFocalLengthAttr(10.0); cam.CreateHorizontalApertureAttr(20.0); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 60))
    _cam_look(cam, (0.20, 0.0, 0.05), pitch_down=15.0)
    son = UsdGeom.Camera.Define(st, root + "/Sonar")                           # sonar head on top, 20 deg down
    Sonar.configure(son)
    _cam_look(son, (0.12, 0.0, 0.16), pitch_down=20.0)
    return op, root + "/Camera", root + "/Sonar"


def _cam_look(cam, pos, pitch_down):
    """USD camera looks down its -Z with +Y up: aim it along local +x, pitched down."""
    p = math.radians(pitch_down)
    f = np.array([math.cos(p), 0.0, -math.sin(p)]); up = np.array([math.sin(p), 0.0, math.cos(p)])
    Zc = -f; Yc = up; Xc = np.cross(Yc, Zc)
    UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d(*Xc, 0, *Yc, 0, *Zc, 0, *pos, 1))


class Sonar:
    """Multibeam imaging sonar from a wide depth render: range from distance_to_camera, intensity from incidence angle
    (Lambert-like backscatter) with Rayleigh speckle, binned into a range x beam polar image."""
    W, H, HFOV, VFOV, BEAMS, BINS = 512, 48, 130.0, 20.0, 256, 400
    RMAX = 12.0

    @classmethod
    def configure(cls, cam):
        f_mm = 18.0 / math.tan(math.radians(cls.HFOV / 2))
        cam.CreateFocalLengthAttr(f_mm); cam.CreateHorizontalApertureAttr(36.0); cam.CreateVerticalApertureAttr(36.0 * cls.H / cls.W)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, cls.RMAX))

    def __init__(self, rng):
        self.rng = rng
        fx = (self.W / 2) / math.tan(math.radians(self.HFOV / 2))
        u = np.arange(self.W) + 0.5 - self.W / 2; v = np.arange(self.H) + 0.5 - self.H / 2
        UU, VV = np.meshgrid(u, v)
        R = np.stack([UU / fx, -VV / fx, -np.ones_like(UU)], -1); self.RAY = R / np.linalg.norm(R, axis=-1, keepdims=True)
        az = np.degrees(np.arctan2(UU, fx))
        self.BEAM = np.clip(((az + self.HFOV / 2) / self.HFOV * self.BEAMS).astype(int), 0, self.BEAMS - 1)
        FW, FH = 560, 300
        fy_, fx_ = np.mgrid[0:FH, 0:FW]
        X = (fx_ - FW / 2) / (FH - 8) * self.RMAX; Y = (FH - 8 - fy_) / (FH - 8) * self.RMAX
        r = np.hypot(X, Y); a = np.degrees(np.arctan2(X, Y))
        self.IN = (r < self.RMAX) & (np.abs(a) < self.HFOV / 2)
        self.RB = (r / self.RMAX * self.BINS).astype(int).clip(0, self.BINS - 1)
        self.AB = ((a + self.HFOV / 2) / self.HFOV * self.BEAMS).astype(int).clip(0, self.BEAMS - 1)
        self.RING = np.zeros(self.IN.shape, bool)
        for rm in (3.0, 6.0, 9.0):
            self.RING |= np.abs(r - rm) < 0.03
        self.R_ = r

    def ping(self, dist, nrm, R_world, pos_world):
        """-> polar uint8 [bins x beams], world points of the returns (N,3)"""
        r = np.asarray(dist, np.float32)
        if r.shape != self.BEAM.shape:
            return None, np.zeros((0, 3), np.float32)
        n = np.asarray(nrm, np.float32)[..., :3]
        ok = np.isfinite(r) & (r > 0.1) & (r < self.RMAX)
        r = np.where(ok, r, 0.0)
        ray_w = self.RAY @ R_world.T
        amp = np.abs(np.sum(ray_w * n, -1)).clip(0.03, 1.0) ** 1.4 * self.rng.rayleigh(0.8, r.shape)
        b = (r / self.RMAX * self.BINS).astype(int).clip(0, self.BINS - 1)
        img = np.zeros((self.BINS, self.BEAMS), np.float32)
        np.add.at(img, (b[ok], self.BEAM[ok]), amp[ok])
        img += self.rng.rayleigh(0.04, img.shape)
        img = np.log1p(img * 6.0); img = (255 * img / max(np.percentile(img, 99.7), 1e-6)).clip(0, 255).astype(np.uint8)
        return img, (pos_world + ray_w[ok] * r[ok][:, None]).astype(np.float32)

    def fan(self, polar):
        g = np.where(self.IN, polar[self.RB, self.AB], 0).astype(np.float32) / 255.0
        rgb = np.stack([g ** 1.6 * 255, g ** 0.9 * 220, g ** 0.7 * 120 + self.IN * 22], -1)      # amber on deep blue
        rgb[self.RING & self.IN] = (60, 90, 110)
        return rgb.clip(0, 255).astype(np.uint8)


def murk(rgb, depth, depth_below_surface, turbidity=0.55):
    """Underwater optics on a clear-air render: Beer-Lambert attenuation per colour channel plus green-brown backscatter
    (Charles River: turbid, visibility ~1-2 m), darker with depth below the surface."""
    d = np.nan_to_num(np.asarray(depth, np.float32), nan=50.0, posinf=50.0)[..., None]
    c = np.array([1.1, 0.55, 0.75], np.float32) * turbidity                      # per-metre attenuation R, G, B
    t = np.exp(-c * d)
    amb = math.exp(-0.35 * max(depth_below_surface, 0.0))
    B = np.array([0.09, 0.16, 0.11], np.float32) * 255 * amb
    out = rgb.astype(np.float32) * t + B * (1 - t)
    return out.clip(0, 255).astype(np.uint8)
