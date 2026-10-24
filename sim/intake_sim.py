"""Intake Sentinel: BlueROV2 Heavy + Reach Alpha 5 + multibeam sonar in the photoreal YC intake scene (live Isaac Sim).

    OMNI_KIT_ACCEPT_EULA=YES ~/IsaacLab/env_isaaclab/bin/python intake_sim.py
    env: CTRL=off (no station keeping)  SECONDS=N (stop after N sim seconds)  HEADLESS=1  NOLOG=1

Scene: the intake channel from ~/biofouling-yc/isaac/render_isaac.py (concrete walls, trash rack, dreissenid clumps as
point instancers on the left wall), real-time RTX with fog instead of path tracing.
Robot: floating articulation (neutral buoyancy, our drag), attitude + station PD on the base, an outer loop keeps the
brush (Alpha 5 TCP) on the wall while the vehicle mows the fouled band; clumps within the brush radius are removed.
Sonar: forward-looking multibeam modelled on a 750 kHz imaging unit (130 x 20 deg, 256 beams, 20 m, 512 range bins).
A wide camera on the ROV gives ray-traced range + normals; each pixel is binned into (beam, range) with incidence
backscatter, Rayleigh speckle and a noise floor -> polar image, fan image, and a per-beam first-return point cloud.
Data: ../data/run_<time>/ sonar_polar/NNNNN.npy (uint8, range x beam), sonar_fan/NNNNN.png, telemetry.csv, clean.csv.
"""
import os, sys, time, math, csv
import numpy as np
from isaacsim.simulation_app import SimulationApp
HEADLESS = os.environ.get("HEADLESS") == "1"
app = SimulationApp({"headless": HEADLESS, "renderer": "RaytracedLighting", "width": 1600, "height": 900})

import carb, omni.usd, omni.ui as ui, omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdShade, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, RigidPrim

H = os.path.dirname(os.path.abspath(__file__))
YC = os.path.expanduser("~/biofouling-yc/isaac")
CTRL = os.environ.get("CTRL", "on") == "on"
DT = 1 / 120

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1 / 30)
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.Xform.Define(st, "/World")

# ---------------------------------------------------------------- photoreal intake scene (shared with the YC film)
sys.path.insert(0, os.path.expanduser("~/glydcar_isaac")); import photoreal as PR
sys.path.insert(0, YC); import state as S
rng = np.random.default_rng(7)
_src = open(os.path.join(YC, "render_isaac.py")).read().splitlines()
_a = next(i for i, l in enumerate(_src) if l.startswith("# ---------------------------------------------------------------- materials"))
_b = next(i for i, l in enumerate(_src) if l.startswith("# frame at which each left-wall clump is cleaned"))
exec("\n".join(_src[_a:_b]), globals())          # materials, walls, floor, trash rack, fouling instancers (TILES)
PR.sky(st, dome_intensity=250.0, sun=(1500.0, (14, -9, 0)), sun_color=(0.9, 0.97, 1.0))
for p in st.Traverse():
    if p.IsA(UsdLux.DomeLight):
        UsdLux.DomeLight(p).GetColorAttr().Set(Gf.Vec3f(0.25, 0.75, 0.7))
cs = carb.settings.get_settings()
for k, v in (("/rtx/fog/enabled", True), ("/rtx/fog/fogColor", [0.02, 0.09, 0.08]), ("/rtx/fog/fogColorIntensity", 1.0),
             ("/rtx/fog/fogStartDist", 0.4), ("/rtx/fog/fogEndDist", 9.0), ("/rtx/post/histogram/enabled", True)):
    cs.set(k, v)
for path in ("/World/Env",):                     # static collision for the vehicle (sonar uses rendering, not physics)
    for p in Usd.PrimRange(st.GetPrimAtPath(path)):
        if p.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(p)

# ---------------------------------------------------------------- robot: BlueROV2 Heavy + Reach Alpha 5
WALL = S.X_WALL                                  # fouled wall plane x (faces +x)
START = np.array([WALL + 0.52, -6.0, 4.3])
rov = st.DefinePrim("/World/ROV", "Xform")
rov.GetReferences().AddReference(os.path.join(H, "..", "assets", "usd", "rov_alpha", "rov_alpha.usda"))
x_ = UsdGeom.Xformable(rov); x_.AddTranslateOp().Set(Gf.Vec3d(*START)); x_.AddRotateZOp().Set(180.0)   # arm faces the wall (-x)
for p in Usd.PrimRange(rov):
    if p.GetName() == "root_joint":
        p.SetActive(False)                       # the importer pins the base to the world
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)       # neutral buoyancy
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        ang = p.IsA(UsdPhysics.RevoluteJoint)
        d = UsdPhysics.DriveAPI.Apply(p, "angular" if ang else "linear")
        d.CreateStiffnessAttr(3.0 if ang else 2000.0); d.CreateDampingAttr(0.3 if ang else 200.0); d.CreateMaxForceAttr(30.0)   # per DEGREE
base = next(p for p in Usd.PrimRange(rov) if p.GetName() == "base_link" and p.HasAPI(UsdPhysics.RigidBodyAPI))
tcp_prim = next(p for p in Usd.PrimRange(rov) if p.GetName() == "alpha_tcp")

d = np.load(os.path.join(YC, "assets", "brov2h.npz"))
CEN = np.array([-0.0095, -0.0005, 0.0125])
hull = UsdGeom.Xform.Define(st, base.GetPath().AppendChild("BROV_visual"))
for k, col in enumerate(d["cols"]):
    P = d[f"P{k}"].astype(np.float64) * 0.025 - CEN; n = len(P) // 3
    m = UsdGeom.Mesh.Define(st, hull.GetPath().AppendChild(f"G{k}"))
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
    m.CreateFaceVertexCountsAttr([3] * n); m.CreateFaceVertexIndicesAttr(list(range(len(P))))
    c = np.asarray(col, float)
    mm = M["blue"] if c[2] > c[0] + 0.15 else (M["acrylic"] if c.mean() > 0.6 else (M["orange"] if c[0] > c[2] + 0.3 else M["frame"]))
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mm)
# work lights (two forward spots, like the BlueROV2 Lumens)
for i, sy in enumerate((-0.12, 0.12)):
    L = UsdLux.SphereLight.Define(st, base.GetPath().AppendChild(f"Lumen{i}"))
    L.CreateRadiusAttr(0.02); L.CreateIntensityAttr(60000.0); L.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))
    UsdLux.ShapingAPI.Apply(L.GetPrim()).CreateShapingConeAngleAttr(35.0)
    lx = UsdGeom.Xformable(L); lx.AddTranslateOp().Set(Gf.Vec3d(0.24, sy, 0.10)); lx.AddRotateYOp().Set(-90.0)

# ---------------------------------------------------------------- multibeam sonar: a wide ray-traced camera on the ROV
SON = dict(beams=256, hfov=130.0, vfov=22.7, rmax=float(os.environ.get("SONAR_RANGE", "5")), bins=512, W=512, Hpx=48)   # 1.2 MHz close-inspection mode: 5 m
f_mm = 18.0 / math.tan(math.radians(SON["hfov"] / 2))
scam = UsdGeom.Camera.Define(st, base.GetPath().AppendChild("Sonar"))
scam.CreateFocalLengthAttr(f_mm); scam.CreateHorizontalApertureAttr(36.0); scam.CreateVerticalApertureAttr(36.0 * SON["Hpx"] / SON["W"])
scam.CreateClippingRangeAttr(Gf.Vec2f(0.05, SON["rmax"]))
# USD cameras look down -Z with +Y up: rows = images of cam X,Y,Z in the hull frame -> look along hull +x, up = hull +z
# mounted on the front corner, looking along the wall (direction of travel, hull -y) and 25 deg in toward it, so the
# fouled wall stretches across the whole 5 m range at grazing incidence (clumps cast acoustic shadows)
_g = math.radians(25.0); _L = np.array([math.sin(_g), -math.cos(_g), 0.0]); _Yc = np.array([0, 0, 1.0]); _Zc = -_L; _Xc = np.cross(_Yc, _Zc)
UsdGeom.Xformable(scam).AddTransformOp().Set(Gf.Matrix4d(*_Xc, 0, *_Yc, 0, *_Zc, 0, 0.20, -0.26, 0.14, 1))
sonar_rp = rep.create.render_product(str(scam.GetPath()), (SON["W"], SON["Hpx"]))
a_dist = rep.AnnotatorRegistry.get_annotator("distance_to_camera"); a_dist.attach([sonar_rp])
a_norm = rep.AnnotatorRegistry.get_annotator("normals"); a_norm.attach([sonar_rp])
if os.environ.get("SONARRGB"):
    a_rgb = rep.AnnotatorRegistry.get_annotator("LdrColor"); a_rgb.attach([rep.create.render_product(str(scam.GetPath()), (1024, 96))])
fx = (SON["W"] / 2) / math.tan(math.radians(SON["hfov"] / 2)); fy = fx
u = (np.arange(SON["W"]) + 0.5 - SON["W"] / 2); v = (np.arange(SON["Hpx"]) + 0.5 - SON["Hpx"] / 2)
UU, VV = np.meshgrid(u, v)
RAY_C = np.stack([UU / fx, -VV / fy, -np.ones_like(UU)], -1); RAY_C /= np.linalg.norm(RAY_C, axis=-1, keepdims=True)
AZ = np.degrees(np.arctan2(UU, fx))                          # beam angle of each pixel column (deg)
BEAM = np.clip(((AZ + SON["hfov"] / 2) / SON["hfov"] * SON["beams"]).astype(int), 0, SON["beams"] - 1)
BEAM_AZ = np.linspace(-SON["hfov"] / 2, SON["hfov"] / 2, SON["beams"])


def ping(cam_R_world, cam_pos=None):
    """one multibeam ping -> (polar uint8 [bins x beams], first-return ranges per beam)"""
    r = np.asarray(a_dist.get_data(), np.float32)
    nrm = np.asarray(a_norm.get_data(), np.float32)[..., :3]
    if r.shape != UU.shape:
        return None, None
    ok = np.isfinite(r) & (r > 0.05) & (r < SON["rmax"])
    ray_w = RAY_C @ cam_R_world.T
    cosi = np.abs(np.sum(ray_w * nrm, -1)).clip(0.02, 1.0)
    amp = cosi ** 1.5                                         # Lambert-like backscatter, rough targets brighter off-normal
    amp *= rng.rayleigh(0.8, amp.shape)                       # speckle
    b = (r / SON["rmax"] * SON["bins"]).astype(int).clip(0, SON["bins"] - 1)
    img = np.zeros((SON["bins"], SON["beams"]), np.float32)
    np.add.at(img, (b[ok], BEAM[ok]), amp[ok])
    img += rng.rayleigh(0.05, img.shape)                      # noise floor
    img = np.log1p(img * 6.0); img = (255 * img / max(np.percentile(img, 99.7), 1e-6)).clip(0, 255).astype(np.uint8)
    first = np.full(SON["beams"], np.nan, np.float32)
    rr = np.where(ok, r, np.inf); col_min = rr.min(0)
    for c in range(SON["W"]):
        if np.isfinite(col_min[c]):
            k = BEAM[0, c]; first[k] = col_min[c] if np.isnan(first[k]) else min(first[k], col_min[c])
    if cam_pos is not None:                                   # 3D returns -> wall mosaic (max height above the wall face)
        Pw = cam_pos + ray_w[ok] * r[ok][:, None]
        hgt = Pw[:, 0] - WALL; sel = (hgt > -0.02) & (hgt < 0.20) & (np.linalg.norm(Pw - ROV_POS[0], axis=1) > 0.75)   # drop own arm/hull
        gy = ((Pw[sel, 1] - MY0) / MRES).astype(int); gz = ((Pw[sel, 2] - MZ0) / MRES).astype(int)
        inb = (gy >= 0) & (gy < MOS.shape[1]) & (gz >= 0) & (gz < MOS.shape[0])
        np.maximum.at(MOS, (gz[inb], gy[inb]), hgt[sel][inb]); np.add.at(HIT, (gz[inb], gy[inb]), 1)
    return img, first


# wall mosaic from sonar returns: fouling height above the concrete face, 4 cm cells over the fouled band
ROV_POS = [np.zeros(3)]
MRES, MY0, MY1, MZ0, MZ1 = 0.04, -7.0, 2.0, 1.0, 5.4
MOS = np.full((int((MZ1 - MZ0) / MRES), int((MY1 - MY0) / MRES)), -1.0, np.float32); HIT = np.zeros(MOS.shape, np.int32)


def mosaic_rgb(w=640):
    h = np.where(HIT > 0, MOS, np.nan)
    g = np.clip(np.nan_to_num(h, nan=0.0) / 0.06, 0, 1)        # 0..6 cm fouling height
    rgb = np.stack([60 + 195 * g, 50 + 120 * g * (1 - g) * 2.5, 70 * (1 - g)], -1)
    rgb[HIT == 0] = (12, 18, 26)
    rgb = rgb[::-1].astype(np.uint8)
    import cv2
    return cv2.resize(rgb, (w, int(w * rgb.shape[0] / rgb.shape[1])), interpolation=cv2.INTER_NEAREST)


FAN_W, FAN_H = 640, 360
_fy, _fx = np.mgrid[0:FAN_H, 0:FAN_W]
_X = (_fx - FAN_W / 2) / (FAN_H - 10) * SON["rmax"]; _Y = (FAN_H - 10 - _fy) / (FAN_H - 10) * SON["rmax"]
_R = np.hypot(_X, _Y); _A = np.degrees(np.arctan2(_X, _Y))
_IN = (_R < SON["rmax"]) & (np.abs(_A) < SON["hfov"] / 2)
_RB = (_R / SON["rmax"] * SON["bins"]).astype(int).clip(0, SON["bins"] - 1)
_AB = ((_A + SON["hfov"] / 2) / SON["hfov"] * SON["beams"]).astype(int).clip(0, SON["beams"] - 1)


def fan(polar):
    g = np.where(_IN, polar[_RB, _AB], 0).astype(np.float32) / 255.0
    rgb = np.stack([g ** 1.6 * 255, g ** 0.9 * 220, g ** 0.7 * 120 + _IN * 18], -1)      # amber-on-blue sonar palette
    for rm in [SON["rmax"] * f for f in (0.25, 0.5, 0.75)]:                                  # range rings
        ring = np.abs(_R - rm) < 0.004 * SON["rmax"]
        rgb[ring & _IN] = (60, 90, 110)
    return rgb.clip(0, 255).astype(np.uint8)


# ---------------------------------------------------------------- viewers
chase = UsdGeom.Camera.Define(st, "/World/Chase"); chase.CreateFocalLengthAttr(16.0); chase.CreateClippingRangeAttr(Gf.Vec2f(0.02, 200))
chase_op = UsdGeom.Xformable(chase).AddTransformOp()
if not HEADLESS:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Chase"
    prov = ui.ByteImageProvider()
    prov2 = ui.ByteImageProvider()
    win = ui.Window("Multibeam sonar (130 x 20 deg, 256 beams, 1.2 MHz inspection model)", width=FAN_W + 20, height=FAN_H + 440)
    with win.frame:
        with ui.VStack():
            ui.ImageWithProvider(prov, width=FAN_W, height=FAN_H)
            ui.Label("Wall mosaic from sonar returns: fouling height 0-6 cm (dark = no data yet)", height=24, style={"font_size": 15, "color": 0xFFBBCCDD})
            ui.ImageWithProvider(prov2, width=FAN_W, height=313)
            stat = ui.Label("", height=40, style={"font_size": 16, "color": 0xFFDDEEFF})

# ---------------------------------------------------------------- data log
RUN = None
if not os.environ.get("NOLOG"):
    RUN = os.path.join(H, "..", "data", "run_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(os.path.join(RUN, "sonar_polar"), exist_ok=True); os.makedirs(os.path.join(RUN, "sonar_fan"), exist_ok=True)
    tel = csv.writer(open(os.path.join(RUN, "telemetry.csv"), "w", newline=""))
    tel.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz", "hold_err_m", "tcp_x", "tcp_y", "tcp_z", "brush_gap_m", "cleared", "total"] +
                 [f"q_{n}" for n in ("e", "d", "c", "b")])
    np.save(os.path.join(RUN, "beam_angles_deg.npy"), BEAM_AZ)
    open(os.path.join(RUN, "sonar_spec.txt"), "w").write(repr(SON) + "\nrange bin = rmax/bins m; polar rows = range bins, cols = beams\n")

# ---------------------------------------------------------------- run
world.reset()
art = next(p for p in Usd.PrimRange(rov) if p.HasAPI(UsdPhysics.ArticulationRootAPI))
robot = Articulation(art.GetPath().pathString); robot.initialize()
B = RigidPrim(base.GetPath().pathString); B.initialize()
idx = {n: i for i, n in enumerate(robot.dof_names)}
A = lambda x: np.asarray(x.cpu() if hasattr(x, "cpu") else x)
Q_D = np.array([0.0, 0.0, 0.0, 1.0])            # (w,x,y,z): yaw 180 deg
TOTAL = sum(len(T[1]) for T in TILES)
GONE = [np.zeros(len(T[1]), bool) for T in TILES]
LANES = [4.3, 3.7, 3.1, 2.5, 1.9]                # mowing rows (base z), fouled band z 1.2..5.0
Y0, Y1, VY = -6.3, 1.3, 0.15
xcmd = START[0]; lane = 0; ydir = 1; ycmd = Y0; zcmd = LANES[0]
steps = int(float(os.environ.get("SECONDS", "0")) / DT) or 10 ** 9
k = 0; nping = 0; t_wall = time.time()


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qrot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


while app.is_running() and k < steps:
    t = k * DT
    # arm: scrub pattern in front of the vehicle (yaw sweep + shoulder nod + wrist spin)
    q = np.zeros(len(idx))
    q[idx["alpha_axis_e"]] = 3.05 + 0.45 * math.sin(1.1 * t)
    q[idx["alpha_axis_d"]] = 1.3 + 0.30 * math.sin(1.7 * t)
    q[idx["alpha_axis_c"]] = 2.6
    q[idx["alpha_axis_b"]] = 1.6 + 1.5 * math.sin(4.0 * t)
    robot.set_joint_position_targets(q.reshape(1, -1))
    # vehicle state
    p, qq = B.get_world_poses(); p = A(p)[0].astype(float); qq = A(qq)[0].astype(float); ROV_POS[0] = p
    v = A(B.get_linear_velocities())[0]; w = A(B.get_angular_velocities())[0]
    tcp = np.array(UsdGeom.Xformable(tcp_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
    gap = tcp[0] - WALL
    # mowing plan + outer loop keeping the brush on the wall
    ycmd += ydir * VY * DT
    if (ydir > 0 and ycmd > Y1) or (ydir < 0 and ycmd < Y0):
        ydir *= -1; lane = (lane + 1) % len(LANES)
    xcmd += -0.8 * (gap - 0.0) * DT                         # push in until the brush touches (1.5 cm target gap)
    xcmd = float(np.clip(xcmd, WALL + 0.30, WALL + 0.80))
    zcmd += float(np.clip(LANES[lane] - zcmd, -0.12 * DT, 0.12 * DT))     # ramp between rows
    tgt = np.array([xcmd, ycmd, zcmd]); vcmd = np.array([0.0, ydir * VY, 0.0])
    f = -25.0 * v + np.array([1.5 * math.sin(0.37 * t), 3.0 * math.sin(0.23 * t), 0.8 * math.sin(0.5 * t)])   # drag + eddies
    tq = -1.5 * w
    if CTRL:
        f += np.clip(260.0 * (tgt - p) + 70.0 * (vcmd - v) + 25.0 * vcmd, -80, 80)   # PD + drag feedforward
        qe = qmul(Q_D, qq * np.array([1, -1, -1, -1]));  qe = qe if qe[0] >= 0 else -qe
        tq += 140.0 * 2.0 * qe[1:] - 30.0 * w
    B.apply_forces_and_torques_at_pos(forces=f.reshape(1, 3), torques=tq.reshape(1, 3), is_global=True)
    world.step(render=(k % 4 == 0) or HEADLESS)
    # cleaning: clumps under the brush (12 cm) come off when the brush is within 9 cm of the wall face (clumps stand 3-6 cm proud)
    if gap < 0.09 and k % 6 == 0:
        for i, T in enumerate(TILES):
            pos = T[1]
            hit = (~GONE[i]) & (np.hypot(pos[:, 1] - tcp[1], pos[:, 2] - tcp[2]) < 0.12)
            if hit.any():
                GONE[i] |= hit
                p2 = pos.copy(); p2[GONE[i], 0] -= 0.3
                T[0].GetPositionsAttr().Set(Vt.Vec3fArray.FromNumpy(p2))
    cleared = int(sum(g.sum() for g in GONE))
    # chase camera: behind and above the vehicle, looking at the wall
    eye = p + np.array([1.9, 1.5, 0.9]); look = np.array([WALL, p[1] - 0.4, p[2] - 0.2])
    m = Gf.Matrix4d(); m.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look), Gf.Vec3d(0, 0, 1)); chase_op.Set(m.GetInverse())
    # sonar ping at 10 Hz
    if k % 12 == 0 and k > 120:
        Rcam = np.array(UsdGeom.Xformable(scam).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractRotationMatrix()).T
        cpos = np.array(UsdGeom.Xformable(scam).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
        img, first = ping(Rcam, cpos)
        if img is not None:
            fimg = fan(img)
            if not HEADLESS:
                rgba = np.concatenate([fimg, np.full(fimg.shape[:2] + (1,), 255, np.uint8)], -1)
                prov.set_data_array(rgba, [FAN_W, FAN_H]) if hasattr(prov, 'set_data_array') else prov.set_bytes_data(list(rgba.tobytes()), [FAN_W, FAN_H])
                if nping % 5 == 0:
                    mo = mosaic_rgb(); mo = np.concatenate([mo, np.full(mo.shape[:2] + (1,), 255, np.uint8)], -1)
                    prov2.set_data_array(mo, [mo.shape[1], mo.shape[0]]) if hasattr(prov2, 'set_data_array') else prov2.set_bytes_data(list(mo.tobytes()), [mo.shape[1], mo.shape[0]])
                stat.text = (f"t {t:5.1f}s   track error {np.linalg.norm((tgt - p)[1:]) * 100:4.1f} cm   brush gap {gap * 100:4.1f} cm   "
                             f"cleared {cleared}/{TOTAL} clumps ({100 * cleared / TOTAL:.1f}%)   station keeping {'ON' if CTRL else 'OFF'}")
            if RUN:
                np.save(os.path.join(RUN, "sonar_polar", f"{nping:05d}.npy"), img)
                np.save(os.path.join(RUN, "sonar_polar", f"{nping:05d}_first.npy"), first)
                if nping % 5 == 0:
                    import cv2; cv2.imwrite(os.path.join(RUN, "sonar_fan", f"{nping:05d}.png"), fimg[:, :, ::-1])
            if os.environ.get("SONARRGB") and nping % 10 == 0:
                import cv2; cv2.imwrite(f"/tmp/claude-1000/sonar_rgb_{nping}.png", np.asarray(a_rgb.get_data())[:, :, 2::-1])
                np.save(f"/tmp/claude-1000/sonar_dist_{nping}.npy", np.asarray(a_dist.get_data()))
            nping += 1
    if RUN and k % 12 == 0:
        jq = A(robot.get_joint_positions())[0]
        tel.writerow([round(t, 3), *np.round(p, 4), *np.round(qq, 5), round(float(np.linalg.norm(tgt - p)), 4), *np.round(tcp, 4),
                      round(float(gap), 4), cleared, TOTAL] + [round(float(jq[idx[n]]), 4) for n in ("alpha_axis_e", "alpha_axis_d", "alpha_axis_c", "alpha_axis_b")])
    if k % 600 == 0:
        print(f"t={t:6.1f}s track={np.linalg.norm((tgt - p)[1:]) * 100:5.1f}cm gap={gap * 100:5.1f}cm cleared={cleared}/{TOTAL} pings={nping} "
              f"rt={t / max(time.time() - t_wall, 1e-6):.2f}x", flush=True)
    k += 1
if RUN:
    with open(os.path.join(RUN, "clean.csv"), "w", newline="") as fo:
        wcsv = csv.writer(fo); wcsv.writerow(["tile", "clumps", "cleared"])
        for T, g in zip(TILES, GONE): wcsv.writerow([str(T[3]), len(g), int(g.sum())])
    np.save(os.path.join(RUN, "wall_mosaic_height_m.npy"), np.where(HIT > 0, MOS, np.nan)); np.save(os.path.join(RUN, "wall_mosaic_hits.npy"), HIT)
    import cv2; cv2.imwrite(os.path.join(RUN, "wall_mosaic.png"), mosaic_rgb(1280)[:, :, ::-1])
    print("DATA", os.path.abspath(RUN), flush=True)
app.close()
