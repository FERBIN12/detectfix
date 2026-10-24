"""BlueROV2 Heavy + Reach Alpha 5 at an intake screen, live in Isaac Sim (real-time RTX).

    OMNI_KIT_ACCEPT_EULA=YES ~/IsaacLab/env_isaaclab/bin/python rov_arm_scene.py          # GUI
    HEADLESS=1 STEPS=600 ... rov_arm_scene.py                                               # smoke test

The ROV floats (neutral buoyancy: gravity off on the robot, linear drag applied by us; PhysX has no water) and a PD
controller holds station in front of the screen while the arm sweeps across it. Mussels are rigid bodies on FixedJoints
with a breakForce: the arm knocks them off. CTRL=off disables station-keeping to show the arm's reaction pushing the ROV.
"""
import os, math
import numpy as np
from isaacsim.simulation_app import SimulationApp
HEADLESS = os.environ.get("HEADLESS") == "1"
app = SimulationApp({"headless": HEADLESS, "width": 1600, "height": 900})

import carb, omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade, PhysxSchema, Gf, Sdf, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, RigidPrim

H = os.path.dirname(os.path.abspath(__file__))
ROV_USD = os.path.join(H, "..", "assets", "usd", "rov_alpha", "rov_alpha.usda")
BROV = os.path.expanduser("~/biofouling-yc/isaac/assets/brov2h.npz")
CTRL = os.environ.get("CTRL", "on") == "on"

world = World(stage_units_in_meters=1.0, physics_dt=1 / 240, rendering_dt=1 / 60)
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdPhysics.Scene.Get(st, "/physicsScene") or None


def mat(path, rgb, rough=0.6, metal=0.0, opacity=None):
    m = UsdShade.Material.Define(st, path)
    s = UsdShade.Shader.Define(st, path + "/S")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m


def cube(path, pos, size, material, dynamic=False, mass=None, collide=True):
    c = UsdGeom.Cube.Define(st, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
        if mass:
            UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)
    return c.GetPrim()


# ---------------------------------------------------------------- light + murky water look
UsdLux.DomeLight.Define(st, "/World/Dome").CreateIntensityAttr(350)
UsdLux.DomeLight.Get(st, "/World/Dome").CreateColorAttr(Gf.Vec3f(0.35, 0.62, 0.62))
sun = UsdLux.DistantLight.Define(st, "/World/Sun")
sun.CreateIntensityAttr(900); sun.CreateColorAttr(Gf.Vec3f(0.55, 0.8, 0.75))
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35, 20, 0))
s = carb.settings.get_settings()
s.set("/rtx/fog/enabled", True)
s.set("/rtx/fog/fogColor", [0.10, 0.24, 0.22])
s.set("/rtx/fog/fogStartDist", 0.4)
s.set("/rtx/fog/fogEndDist", 4.0)
s.set("/rtx/fog/fogColorIntensity", 1.0)

# ---------------------------------------------------------------- intake screen: frame, rails, mesh, mussels
M_STEEL = mat("/World/M/steel", (0.42, 0.44, 0.42), 0.45, 0.8)
M_RUST = mat("/World/M/rust", (0.35, 0.22, 0.12), 0.8, 0.3)
M_MUSSEL = mat("/World/M/mussel", (0.08, 0.07, 0.06), 0.35)
M_SAND = mat("/World/M/sand", (0.30, 0.28, 0.20), 0.95)
SX = 0.62                                    # screen face x (ROV at origin faces +x)
cube("/World/Floor", (0, 0, -0.9), (8, 8, 0.05), M_SAND)
screen = cube("/World/Screen/Panel", (SX + 0.02, 0, 0), (0.04, 1.6, 1.2), M_STEEL)
for i, y in enumerate((-0.8, -0.27, 0.27, 0.8)):          # rails split 3 bays
    cube(f"/World/Screen/Rail{i}", (SX - 0.03, y, 0), (0.06, 0.05, 1.2), M_RUST)
for i, z in enumerate((-0.6, 0.6)):
    cube(f"/World/Screen/Bar{i}", (SX - 0.03, 0, z), (0.06, 1.6, 0.05), M_RUST)
rng = np.random.default_rng(3)
MUSSELS = []
for b, (y0, y1) in enumerate(((-0.77, -0.30), (-0.24, 0.24), (0.30, 0.77))):
    for k in range(18):
        y, z = rng.uniform(y0, y1), rng.uniform(-0.45, 0.35)
        p = cube(f"/World/Mussels/b{b}_{k}", (SX - 0.025, y, z), (0.03, 0.05, 0.025), M_MUSSEL, dynamic=True, mass=0.03)
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)   # held by the joint until it breaks
        j = UsdPhysics.FixedJoint.Define(st, f"/World/MusselJoints/b{b}_{k}")
        j.CreateBody0Rel().SetTargets([screen.GetPath()])
        j.CreateBody1Rel().SetTargets([p.GetPath()])
        # panel is scaled (0.04,1.6,1.2): local coords are divided by the scale
        j.CreateLocalPos0Attr(Gf.Vec3f(-0.045 / 0.04, y / 1.6, z / 1.2))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateBreakForceAttr(25.0 + 15.0 * b)               # bay 0 light .. bay 2 heavy fouling
        j.CreateBreakTorqueAttr(2.0 + b)
        MUSSELS.append(p.GetPath().pathString)

# ---------------------------------------------------------------- the robot
rov = st.DefinePrim("/World/ROV", "Xform")
rov.GetReferences().AddReference(os.path.abspath(ROV_USD))
UsdGeom.Xformable(rov).AddTranslateOp().Set(Gf.Vec3d(0.17, 0.45, 0.05))
for p in Usd.PrimRange(rov):
    if p.GetName() == "root_joint":                 # importer fixes the base to the world: the ROV must float
        p.SetActive(False)
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)   # neutral buoyancy
        PhysxSchema.PhysxRigidBodyAPI(p).CreateLinearDampingAttr(0.0)
base = next(p for p in Usd.PrimRange(rov) if p.GetName() == "base_link" and p.HasAPI(UsdPhysics.RigidBodyAPI))
for p in Usd.PrimRange(rov):                         # arm joints: stiff position drives (servo arm)
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        kind = "angular" if p.IsA(UsdPhysics.RevoluteJoint) else "linear"
        d = UsdPhysics.DriveAPI.Apply(p, kind)
        d.CreateStiffnessAttr(3.0 if kind == "angular" else 2000.0)      # angular drives are per DEGREE in USD
        d.CreateDampingAttr(0.3 if kind == "angular" else 200.0)
        d.CreateMaxForceAttr(30.0)

# BlueROV2 Heavy hull visual (our existing mesh, Blue Robotics frame: +x forward), under the floating base link
d = np.load(BROV)
M_FRAME = mat("/World/M/brov_black", (0.03, 0.03, 0.035), 0.4)
M_BLUE = mat("/World/M/brov_blue", (0.05, 0.25, 0.75), 0.35)
M_CLEAR = mat("/World/M/brov_tube", (0.75, 0.8, 0.82), 0.1, 0.0)
CEN = np.array([-0.0095, -0.0005, 0.0125])
hull = UsdGeom.Xform.Define(st, base.GetPath().AppendChild("BROV_visual"))
for k, col in enumerate(d["cols"]):
    P = d[f"P{k}"].astype(np.float64) * 0.025 - CEN
    n = len(P) // 3
    m = UsdGeom.Mesh.Define(st, hull.GetPath().AppendChild(f"G{k}"))
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
    m.CreateFaceVertexCountsAttr([3] * n)
    m.CreateFaceVertexIndicesAttr(list(range(len(P))))
    c = np.asarray(col, float)
    mm = M_BLUE if (c[2] > c[0] + 0.15) else (M_CLEAR if c.mean() > 0.6 else M_FRAME)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mm)

# ---------------------------------------------------------------- camera
cam = UsdGeom.Camera.Define(st, "/World/ChaseCam")
cam.CreateFocalLengthAttr(18.0)
cx = UsdGeom.Xformable(cam)
cx.AddTranslateOp().Set(Gf.Vec3d(-0.9, -1.25, 0.55))
cx.AddRotateXYZOp().Set(Gf.Vec3f(72, 0, -38))
if not HEADLESS:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/ChaseCam"

# ---------------------------------------------------------------- run
world.reset()
art_root = next(p for p in Usd.PrimRange(rov) if p.HasAPI(UsdPhysics.ArticulationRootAPI))
robot = Articulation(art_root.GetPath().pathString)
robot.initialize()
names = robot.dof_names
print("DOFS", names, flush=True)
B = RigidPrim(base.GetPath().pathString)
B.initialize()
Mz = RigidPrim("/World/Mussels/.*")
Mz.initialize()
m0, _ = Mz.get_world_poses()
m0 = np.asarray(m0.cpu() if hasattr(m0, "cpu") else m0)
HOME = np.array([0.17, 0.45, 0.05])


def a(x):
    return np.asarray(x.cpu() if hasattr(x, "cpu") else x)


idx = {n: i for i, n in enumerate(names)}
steps = int(os.environ.get("STEPS", "0")) or 10 ** 9
t, k = 0.0, 0
while app.is_running() and k < steps:
    t = k * world.get_physics_dt()
    # arm: reach toward the screen and sweep side to side (a scrubbing pass)
    q = np.zeros(len(names))
    if "alpha_axis_e" in idx: q[idx["alpha_axis_e"]] = 3.05 + 0.55 * math.sin(0.5 * t)       # base yaw sweep
    if "alpha_axis_d" in idx: q[idx["alpha_axis_d"]] = 1.3 + 0.35 * math.sin(1.3 * t)       # shoulder
    if "alpha_axis_c" in idx: q[idx["alpha_axis_c"]] = 2.6
    if "alpha_axis_b" in idx: q[idx["alpha_axis_b"]] = 1.6 + 1.5 * math.sin(3.0 * t)              # wrist roll = brush spin
    robot.set_joint_position_targets(q.reshape(1, -1))
    # water + station keeping on the floating base
    p, _ = B.get_world_poses(); p = a(p)[0]
    v = a(B.get_linear_velocities())[0]
    w = a(B.get_angular_velocities())[0]
    f = -18.0 * v                                                        # linear drag
    f += np.array([-6.0, 2.5 * math.sin(0.9 * t), 0.0])                 # eddies + arm reaction push
    tq = -2.0 * w
    if CTRL:
        f += np.clip(220.0 * (HOME - p) - 60.0 * v, -60, 60)
        tq += -12.0 * w
    B.apply_forces_and_torques_at_pos(forces=f.reshape(1, 3), torques=tq.reshape(1, 3), is_global=True)
    world.step(render=not HEADLESS)
    k += 1
    if k % 240 == 0:
        mp = a(Mz.get_world_poses()[0])
        off = int(np.sum(np.linalg.norm(mp - m0, axis=1) > 0.02))
        tcp = next((pp for pp in Usd.PrimRange(rov) if pp.GetName() == "alpha_tcp"), None)
        tp = UsdGeom.Xformable(tcp).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation() if tcp else None
        print(f"tcp={tuple(round(x, 2) for x in tp) if tp else None} base={tuple(p.round(2))}", flush=True)
        print(f"t={t:5.1f}s hold_err={np.linalg.norm(HOME - p) * 100:5.1f} cm  mussels_off={off}/{len(m0)}", flush=True)
app.close()
