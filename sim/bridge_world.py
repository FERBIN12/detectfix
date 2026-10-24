"""Bridge world: a photoreal 3-span concrete highway bridge over a Massachusetts river, inspected by a quadcopter.

    OMNI_KIT_ACCEPT_EULA=YES ~/IsaacLab/env_isaaclab/bin/python bridge_world.py              # live (real-time RTX)
    HEADLESS=1 ~/IsaacLab/env_isaaclab/bin/python bridge_world.py                             # record one flight
    STILL=1 HEADLESS=1 ...  (path-traced hero stills of the bridge into data/stills/)

World (metres, Z up): river runs along +Y, the bridge spans X from -60 to +60, deck top at z=10.5, water at z=0.
Defects (cracks, spalls with exposed rebar, efflorescence, rust streaks) are geometry on the piers and girders, each
with a ground-truth record in data/defects_gt.json. The drone flies a kinematic inspection route that keeps a fixed
standoff from each face; its gimbal camera writes RGB + pose + target element per waypoint to data/flight_<time>/.
"""
import os, sys, math, json, time
import numpy as np
from isaacsim.simulation_app import SimulationApp
HEADLESS = os.environ.get("HEADLESS") == "1"
STILL = os.environ.get("STILL") == "1"
app = SimulationApp({"headless": HEADLESS, "renderer": "PathTracing" if STILL else "RaytracedLighting",
                     "width": 1600, "height": 900})

import carb, omni.usd, omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Gf, Sdf, Vt
sys.path.insert(0, os.path.expanduser("~/glydcar_isaac")); import photoreal as PR

H = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(H, "..", "data"); os.makedirs(DATA, exist_ok=True)
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.Xform.Define(st, "/World")
rng = np.random.default_rng(24)

# ---------------------------------------------------------------- materials
M = dict(
    pier=PR.base(st, "/World/M/pier", "Masonry", "Concrete_Formed", tile_m=2.0),
    girder=PR.base(st, "/World/M/girder", "Masonry", "Concrete_Smooth", tile_m=2.5),
    deck=PR.base(st, "/World/M/deck", "Masonry", "Concrete_Rough", tile_m=2.5),
    asphalt=PR.base(st, "/World/M/asphalt", "Natural", "Asphalt", tile_m=3.0),
    grass=PR.base(st, "/World/M/grass", "Natural", "Grass_Countryside", tile_m=4.0),
    soil=PR.base(st, "/World/M/soil", "Natural", "Soil_Rocky", tile_m=3.0),
    # turbid water from the Harbour USV film (Fresnel gloss over a silty body + animated ripple normal maps)
    water=PR.mdl_material(st, "/World/M/water", os.path.expanduser("~/harbour-usv/isaac/mdl/HarbourWater.mdl"), "HarbourWater",
                          {"body_color": ("c3", (0.034, 0.042, 0.030)), "roughness": 0.03, "normal_strength": 1.7}),
    steel=PR.base(st, "/World/M/steel", "Metals", "Steel_Carbon", tile_m=1.0),
    rebar=PR.base(st, "/World/M/rebar", "Metals", "RustedMetal", tile_m=0.3),
    paint=PR.omnipbr(st, "/World/M/paint", (0.85, 0.85, 0.80), 0.5),
    yellow=PR.omnipbr(st, "/World/M/yellow", (0.85, 0.62, 0.05), 0.5),
    crack=PR.omnipbr(st, "/World/M/crack", (0.015, 0.014, 0.012), 0.9),
    spall=PR.base(st, "/World/M/spall", "Natural", "Soil_Rocky", tile_m=0.4),
    efflor=PR.omnipbr(st, "/World/M/efflor", (0.86, 0.85, 0.80), 0.95),
    rust=PR.omnipbr(st, "/World/M/rust", (0.36, 0.17, 0.06), 0.85),
    stain=PR.omnipbr(st, "/World/M/stain", (0.20, 0.19, 0.16), 0.9),
    algae=PR.omnipbr(st, "/World/M/algae", (0.07, 0.075, 0.045), 0.85),
    riprap=PR.base(st, "/World/M/riprap", "Stone", "Gravel_River_Rock", tile_m=1.2),
    pad=PR.omnipbr(st, "/World/M/pad", (0.03, 0.03, 0.03), 0.7),
)


def bind(prim, m):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(m)


def box(path, center, size, m, rot_z=0.0):
    c = UsdGeom.Cube.Define(st, path); c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c); x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if rot_z: x.AddRotateZOp().Set(float(rot_z))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size))); bind(c.GetPrim(), m)
    return c.GetPrim()


def cyl(path, center, r, h, m, axis="Z"):
    c = UsdGeom.Cylinder.Define(st, path); c.CreateRadiusAttr(float(r)); c.CreateHeightAttr(float(h)); c.CreateAxisAttr(axis)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*map(float, center))); bind(c.GetPrim(), m)
    return c.GetPrim()


def mesh(path, P, F, m):
    me = UsdGeom.Mesh.Define(st, path)
    me.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(P, np.float32)))
    me.CreateFaceVertexCountsAttr([len(f) for f in F]); me.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    me.CreateSubdivisionSchemeAttr("none"); bind(me.GetPrim(), m)
    return me


# ---------------------------------------------------------------- terrain + river
HALF = 64.5                                   # half bridge length (set again with the bridge below)
BED, BANK_TOP = -3.5, 5.0
def ground_z(x, y):
    ax = np.abs(x)
    t = np.clip((ax - (HALF - 6)) / 20.0, 0, 1); t = t * t * (3 - 2 * t)         # channel to the abutments, banks rise behind them
    z = BED + (BANK_TOP - BED) * t
    z += 0.35 * np.sin(0.11 * y + 0.7) * np.cos(0.07 * x) + 0.25 * np.sin(0.23 * x + 0.05 * y)
    return z

N = 140; xs = np.linspace(-150, 150, N); ys = np.linspace(-150, 150, N)
X, Y = np.meshgrid(xs, ys); Z = ground_z(X, Y)
P = np.stack([X, Y, Z], -1).reshape(-1, 3)
Fg, Fs, Fr = [], [], []
for j in range(N - 1):
    for i in range(N - 1):
        a = j * N + i; f = [a, a + 1, a + N + 1, a + N]
        (Fs if Z[j, i] < -0.6 else (Fr if Z[j, i] < 1.6 else Fg)).append(f)       # riverbed soil, riprap band at the waterline, grass above
mesh("/World/Terrain/Grass", P, Fg, M["grass"]); mesh("/World/Terrain/Soil", P, Fs, M["soil"]); mesh("/World/Terrain/Riprap", P, Fr, M["riprap"])
RN = (130, 300); rx = np.linspace(-(HALF + 2), HALF + 2, RN[0]); ry = np.linspace(-150, 150, RN[1]); RX, RY = np.meshgrid(rx, ry)
RZ = 0.04 * np.sin(0.35 * RY + 0.2 * RX) + 0.02 * np.sin(0.8 * RY - 0.5 * RX + 1.3)   # slow swell; fine ripples come from the normal maps
RF = [[j * RN[0] + i, j * RN[0] + i + 1, (j + 1) * RN[0] + i + 1, (j + 1) * RN[0] + i] for j in range(RN[1] - 1) for i in range(RN[0] - 1)]
_rv = mesh("/World/River", np.stack([RX, RY, RZ], -1).reshape(-1, 3), RF, M["water"])
_gy, _gx = np.gradient(RZ, ry, rx); _n = np.stack([-_gx, -_gy, np.ones_like(RZ)], -1); _n /= np.linalg.norm(_n, axis=-1, keepdims=True)
_rv.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(_n.reshape(-1, 3).astype(np.float32))); _rv.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
WTILE, WTEX = 4.0, os.path.expanduser("~/harbour-usv/isaac/tex")                 # ripple normal-map tile (m) and frames
UsdGeom.PrimvarsAPI(_rv.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
    Vt.Vec2fArray.FromNumpy((np.stack([RX, RY], -1).reshape(-1, 2) / WTILE).astype(np.float32)))
_wn = UsdShade.Shader.Get(st, "/World/M/water/Shader").CreateInput("normal_map_texture", Sdf.ValueTypeNames.Asset)
def set_water_normal(f):
    _wn.Set(Sdf.AssetPath(os.path.join(WTEX, f"water_n{f % 96:02d}.png"))); _wn.GetAttr().SetColorSpace("raw")
set_water_normal(0)

# ---------------------------------------------------------------- bridge (spans X, deck top z=10.5)
L, W = 129.0, 15.0                    # NBI B1601637HMUNNBI: N Washington St over the Charles, 5 spans, 129 m
HALF = L / 2
PIERS = [-38.7, -12.9, 12.9, 38.7]
DECK_TOP, DECK_T = 10.5, 0.25
GIRDER_D = 1.6
G_TOP = DECK_TOP - DECK_T                       # girder top
G_BOT = G_TOP - GIRDER_D
CAP_TOP = G_BOT; CAP_H = 1.4
GY = np.linspace(-5.6, 5.6, 5)                   # 5 girders
ELEMENTS = {}

# abutments into the banks
for s in (-1, 1):
    x0 = s * HALF
    zb = float(ground_z(np.array(x0), np.array(0.0)))
    box(f"/World/Bridge/Abutment{s+1}", (x0 + s * 1.5, 0, (zb - 2 + CAP_TOP) / 2), (3.0, W + 1.0, CAP_TOP - zb + 2), M["pier"])
    box(f"/World/Bridge/Wingwall{s+1}a", (x0 + s * 5, W / 2 + 0.6, (zb + CAP_TOP) / 2), (10, 0.6, CAP_TOP - zb + 1), M["pier"])
    box(f"/World/Bridge/Wingwall{s+1}b", (x0 + s * 5, -W / 2 - 0.6, (zb + CAP_TOP) / 2), (10, 0.6, CAP_TOP - zb + 1), M["pier"])
    ELEMENTS[f"abutment_{'W' if s < 0 else 'E'}"] = (x0, 0, CAP_TOP - 2)

# piers: footing, 3 columns, cap beam
for k, px in enumerate(PIERS):
    box(f"/World/Bridge/Pier{k}/Footing", (px, 0, BED - 0.5), (4.5, 13.0, 2.0), M["pier"])
    for j, cy in enumerate((-4.6, 0.0, 4.6)):
        cyl(f"/World/Bridge/Pier{k}/Col{j}", (px, cy, (BED + 0.5 + CAP_TOP - CAP_H) / 2), 0.75, CAP_TOP - CAP_H - BED - 0.5, M["pier"])
        cyl(f"/World/Bridge/Pier{k}/Algae{j}", (px, cy, 0.25), 0.765, 1.3, M["algae"])               # wet/algae band at the waterline
    box(f"/World/Bridge/Pier{k}/Cap", (px, 0, CAP_TOP - CAP_H / 2), (2.2, W - 0.6, CAP_H), M["pier"])
    for g, gy in enumerate(GY):
        for sx in (-0.6, 0.6):
            box(f"/World/Bridge/Pier{k}/Pad{g}_{int(sx > 0)}", (px + sx, gy, CAP_TOP + 0.04), (0.45, 0.6, 0.08), M["pad"])
    ELEMENTS[f"pier_{k+1}"] = (px, 0, (BED + CAP_TOP) / 2)

# girders: precast I-beams (two flanges + web), one per bay
supports = [-HALF, *PIERS, HALF]
for b in range(len(supports) - 1):
    x0, x1 = supports[b] + 0.4, supports[b + 1] - 0.4; cx, lx = (x0 + x1) / 2, x1 - x0
    for g, gy in enumerate(GY):
        base = f"/World/Bridge/Span{b}/G{g}"
        box(base + "/Top", (cx, gy, G_TOP - 0.1), (lx, 1.0, 0.2), M["girder"])
        box(base + "/Web", (cx, gy, (G_TOP + G_BOT) / 2), (lx, 0.2, GIRDER_D - 0.4), M["girder"])
        box(base + "/Bot", (cx, gy, G_BOT + 0.15), (lx, 0.65, 0.3), M["girder"])
    for di, dx in enumerate(np.linspace(x0 + 1, x1 - 1, 4)):                     # diaphragms
        box(f"/World/Bridge/Span{b}/Diaph{di}", (dx, 0, (G_TOP + G_BOT) / 2), (0.25, GY[-1] - GY[0], GIRDER_D - 0.5), M["girder"])
    ELEMENTS[f"span_{b+1}"] = (cx, 0, G_BOT)

# deck, overhangs, parapets, road surface, lane lines, light poles
box("/World/Bridge/Deck", (0, 0, DECK_TOP - DECK_T / 2), (L + 6, W, DECK_T), M["deck"])
box("/World/Bridge/Road", (0, 0, DECK_TOP + 0.04), (L + 6, W - 1.6, 0.08), M["asphalt"])
for s in (-1, 1):
    box(f"/World/Bridge/Parapet{s+1}", (0, s * (W / 2 - 0.35), DECK_TOP + 0.5), (L + 6, 0.5, 1.0), M["girder"])
    box(f"/World/Bridge/Edge{s+1}", (0, s * (W / 2 - 1.25), DECK_TOP + 0.085), (L + 6, 0.15, 0.01), M["paint"])
    for k, x in enumerate(np.arange(-HALF + 5, HALF, 21.5)):
        cyl(f"/World/Bridge/Pole{s+1}_{k}", (x, s * (W / 2 - 0.35), DECK_TOP + 5.0), 0.1, 8.0, M["steel"])
        box(f"/World/Bridge/Arm{s+1}_{k}", (x, s * (W / 2 - 1.5), DECK_TOP + 8.9), (0.15, 2.4, 0.12), M["steel"])
box("/World/Bridge/Center", (0, 0, DECK_TOP + 0.085), (L + 6, 0.3, 0.01), M["yellow"])
for xi, x in enumerate(np.arange(-HALF, HALF + 2, 6.0)):
    for yi, y in enumerate((-3.4, 3.4)):
        box(f"/World/Bridge/Dash{xi}_{yi}", (x, y, DECK_TOP + 0.085), (3.0, 0.15, 0.01), M["paint"])

# ---------------------------------------------------------------- defects (ground truth for the AI)
DEFECTS = []
def defect(kind, element, pos, severity, normal):
    n = np.asarray(normal, float); p = np.asarray(pos, float); did = f"D{len(DEFECTS)+1:02d}"
    root = f"/World/Defects/{did}"
    off = p + n * 0.02
    if kind == "crack":                                                              # jagged dark polyline
        pts = [off]; d = np.array([0, 0, -1.0]) if abs(n[2]) < 0.5 else np.array([1.0, 0, 0])
        side = np.cross(n, d)
        for i in range(14):
            pts.append(pts[-1] + d * rng.uniform(0.12, 0.22) + side * rng.uniform(-0.08, 0.08))
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; mid = (a + b) / 2; ln = np.linalg.norm(b - a)
            ang = math.degrees(math.atan2((b - a)[1], (b - a)[0]))
            c = UsdGeom.Cube.Define(st, f"{root}/s{i}"); c.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(c); xf.AddTranslateOp().Set(Gf.Vec3d(*mid))
            m = Gf.Matrix3d(*(np.array([b - a, side, n]).T / np.array([ln, 1, 1])).T.flatten()) if False else None
            vec = (b - a) / ln
            rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*vec))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(rot.GetQuat()))
            xf.AddScaleOp().Set(Gf.Vec3f(float(ln), 0.012 + 0.01 * severity, 0.03)); bind(c.GetPrim(), M["crack"])
    elif kind == "spall":                                                           # rough patch + exposed rebar
        tang = np.cross(n, [0, 0, 1.0]); tang = tang / (np.linalg.norm(tang) + 1e-9) if np.linalg.norm(tang) > 1e-6 else np.array([1.0, 0, 0])
        up = np.cross(tang, n); rr = 0.16 + 0.07 * severity
        ring = [p + n * 0.015 + rr * (math.cos(a) * tang + 0.7 * math.sin(a) * up) * rng.uniform(0.55, 1.25) for a in np.linspace(0, 6.283, 14, endpoint=False)]
        mesh(f"{root}/patch", [p + n * 0.02] + ring, [[0, i + 1, (i + 1) % 14 + 1] for i in range(14)], M["spall"])
        for i, o in enumerate((-0.12, 0.12)):
            c0 = p + n * 0.05 + up * o
            r = UsdGeom.Cylinder.Define(st, f"{root}/bar{i}"); r.CreateRadiusAttr(0.014); r.CreateHeightAttr(float(rr * 1.6)); r.CreateAxisAttr("X")
            xf = UsdGeom.Xformable(r); xf.AddTranslateOp().Set(Gf.Vec3d(*c0))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*tang)).GetQuat()))
            bind(r.GetPrim(), M["rebar"])
    else:                                                                           # efflorescence / rust streak: tall stain
        tang = np.cross(n, [0, 0, 1.0]); tang = tang / (np.linalg.norm(tang) + 1e-9)
        w, h = (0.5 + 0.2 * severity, 1.4 + 0.5 * severity) if kind == "efflorescence" else (0.18, 1.8 + 0.4 * severity)
        q = [p + n * 0.012 + tang * sx * w / 2 + np.array([0, 0, 1.0]) * sz for sx, sz in ((-1, 0), (1, 0), (0.6, -h), (-0.6, -h))]
        mesh(f"{root}/stain", q, [[0, 1, 2, 3]], M["efflor" if kind == "efflorescence" else "rust"])
    DEFECTS.append(dict(id=did, type=kind, severity=int(severity), element=element, position=[round(float(v), 3) for v in p],
                        normal=[float(v) for v in n]))

C_R = 0.75
# pier 2: spall at the waterline column, crack on the cap; pier 3: efflorescence + rust streak + crack
defect("spall", "pier_2", (PIERS[1] + C_R, -4.6, 1.2), 3, (1, 0, 0))
defect("crack", "pier_2", (PIERS[1] + 1.1, 2.0, CAP_TOP - 0.2), 2, (1, 0, 0))
defect("crack", "pier_2", (PIERS[1] - C_R, 0.0, 5.5), 1, (-1, 0, 0))
defect("efflorescence", "pier_3", (PIERS[2] - 1.1, -3.0, CAP_TOP - 0.2), 2, (-1, 0, 0))
defect("rust_streak", "pier_3", (PIERS[2] - C_R, 4.6, 6.5), 2, (-1, 0, 0))
defect("crack", "pier_3", (PIERS[2] + C_R, -4.6, 3.0), 3, (1, 0, 0))
# girders: spall at a bearing end (span 2), cracks near midspan bottom flange (span 1), rust at a joint (span 3)
defect("spall", "span_3", (PIERS[1] + 1.2, GY[1] - 0.33, G_BOT + 0.6), 2, (0, -1, 0))
defect("crack", "span_2", (-26.0, GY[4] + 0.33, G_BOT + 0.5), 1, (0, 1, 0))
defect("rust_streak", "span_4", (PIERS[2] + 1.0, GY[3] + 0.11, G_TOP - 0.45), 1, (0, 1, 0))
json.dump(dict(bridge="Charles River bridge modelled on the NBI 2022 record for the N Washington St (Charlestown) Bridge: 5 spans, 129 m", defects=DEFECTS),
          open(os.path.join(DATA, "defects_gt.json"), "w"), indent=1)

# ---------------------------------------------------------------- vegetation, rocks, sky
SPECIES = ["Red_Maple", "Black_Oak_Fall", "Scarlet_Oak_fall", "Gray_Birch_fall", "Red_Ash_Fall", "Eastern_Hemlock", "Honey_Locust_Fall"]
SHRUBS = ["Burning_Bush", "Juniper", "Holly", "Fountain_Grass_Tall", "Grass_Short_A", "Pampas_Grass"]
k = 0
for s in (-1, 1):
    for y in np.arange(-140, 141, 8.0):
        if abs(y) < 14: continue
        for band in (0, 1, 2):
            x = s * (HALF + 3 + band * 12 + rng.uniform(-4, 4)); yy = y + rng.uniform(-3, 3)
            t = st.DefinePrim(f"/World/Trees/T{k}", "Xform")
            UsdGeom.Xformable(t).AddTranslateOp().Set(Gf.Vec3d(0, 0, float(ground_z(np.array(x), np.array(yy))) - 0.2))
            PR.tree(st, f"/World/Trees/T{k}/tree", SPECIES[int(rng.integers(len(SPECIES)))], (float(x), float(yy)), float(rng.uniform(0, 360)), float(rng.uniform(0.8, 1.25))); k += 1
for i in range(110):                                                               # shrubs + tall grass on the banks
    s = int(rng.choice([-1, 1])); x = s * rng.uniform(HALF - 14, HALF + 2); y = rng.uniform(-120, 120)
    if abs(y) < 10: continue
    t = st.DefinePrim(f"/World/Shrubs/S{i}", "Xform")
    UsdGeom.Xformable(t).AddTranslateOp().Set(Gf.Vec3d(0, 0, float(ground_z(np.array(x), np.array(y))) - 0.05))
    w = UsdGeom.Xform.Define(st, f"/World/Shrubs/S{i}/w"); wx = UsdGeom.Xformable(w)        # shrub assets carry their own xform ops
    wx.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0)); wx.AddRotateZOp().Set(float(rng.uniform(0, 360)))
    wx.AddScaleOp().Set(Gf.Vec3f(*(3 * [float(rng.uniform(0.7, 1.3)) * PR.VEG_UNIT])))
    sp = st.DefinePrim(f"/World/Shrubs/S{i}/w/p", "Xform"); sp.GetReferences().AddReference(f"{PR.VEG}/Shrub/{SHRUBS[int(rng.integers(len(SHRUBS)))]}.usd"); sp.SetInstanceable(True)
ROCK = PR.VEG + "/Rocks/rock_small_{:02d}.usda"
def rock(path, xyz, sc):
    r = UsdGeom.Xform.Define(st, path).GetPrim()
    rc = st.DefinePrim(path + "/r", "Xform"); rc.GetReferences().AddReference(ROCK.format(int(rng.integers(1, 16)))); rc.SetInstanceable(True)
    xf = UsdGeom.Xformable(r); xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, xyz))); xf.AddRotateZOp().Set(float(rng.uniform(0, 360)))
    xf.AddScaleOp().Set(Gf.Vec3f(*(3 * [float(sc) * PR.VEG_UNIT])))
for i in range(220):                                                               # riprap boulders along both banks
    s = int(rng.choice([-1, 1])); x = s * rng.uniform(HALF - 5.5, HALF - 1); y = rng.uniform(-90, 90)
    rock(f"/World/Riprap/R{i}", (x, y, float(ground_z(np.array(x), np.array(y))) - 0.1), rng.uniform(4, 9))
for k_, px in enumerate(PIERS):                                                    # scour-protection stone around the footings
    for i in range(26):
        a = rng.uniform(0, 6.283); rr = rng.uniform(2.6, 4.2)
        rock(f"/World/PierStone/P{k_}_{i}", (px + 1.6 * math.cos(a) * rr / 2.6, 6.8 * math.sin(a) * rr / 3.5, BED + 0.4), rng.uniform(5, 10))
PR.sky(st, hdr="Cloudy/kloofendal_48d_partly_cloudy_4k.hdr", dome_intensity=900.0, sun=(3200.0, (-40, 25, 140)))

# ---------------------------------------------------------------- Boston-style city blocks on the horizon (North End / West End side)
SKY_M = [PR.base(st, "/World/M/bld0", "Masonry", "Brick_Wall_Red", tile_m=3.0), PR.base(st, "/World/M/bld1", "Masonry", "Concrete_Block", tile_m=3.0),
         PR.base(st, "/World/M/bld2", "Masonry", "Brick_Wall_Brown", tile_m=3.0), PR.omnipbr(st, "/World/M/bld3", (0.10, 0.13, 0.16), 0.08, 0.6),
         PR.omnipbr(st, "/World/M/bld4", (0.55, 0.57, 0.6), 0.2, 0.5)]
bi = 0
for ang in np.linspace(0, 2 * math.pi, 140, endpoint=False):
    r = rng.uniform(520, 820); cx, cy = r * math.cos(ang), r * math.sin(ang)
    south = cy < -50                                                          # downtown side: taller towers
    h = rng.uniform(40, 150) if south and rng.random() < 0.5 else rng.uniform(12, 35)
    w_ = rng.uniform(18, 45)
    box(f"/World/City/B{bi}", (cx, cy, float(ground_z(np.array(cx), np.array(cy))) + h / 2 - 1), (w_, rng.uniform(18, 45), h),
        SKY_M[3 if h > 45 else int(rng.choice([0, 2, 3, 4]))], rot_z=math.degrees(ang)); bi += 1
box("/World/City/Arena", (-90, -560, 18), (130, 90, 34), SKY_M[4])            # TD Garden-sized arena block, south bank

# ---------------------------------------------------------------- traffic on the deck (moves in the live view)
CAR_PROTOS = PR.parked_car_protos(st)
CARS = []
for i in range(10):
    lane = [-5.1, -1.7, 1.7, 5.1][i % 4]; dirn = 1 if lane > 0 else -1
    root = st.DefinePrim(f"/World/Traffic/Car{i}", "Xform"); op = UsdGeom.Xformable(root).AddTransformOp()
    PR.parked_car(st, f"/World/Traffic/Car{i}/c", CAR_PROTOS[i % len(CAR_PROTOS)], (0, 0), 0 if dirn > 0 else 180)
    CARS.append([op, lane, dirn, rng.uniform(-90, 90), rng.uniform(18, 27)])    # op, y, direction, x, speed m/s
    op.Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(CARS[-1][3], lane, DECK_TOP + 0.08)))

# ---------------------------------------------------------------- inspection drone (Matrice-class quad, ~0.9 m diagonal)
D = dict(body=PR.omnipbr(st, "/World/M/d_body", (0.12, 0.12, 0.13), 0.35, 0.2), arm=PR.base(st, "/World/M/d_cf", "Metals", "Aluminum_Anodized_Charcoal", tile_m=0.2),
         motor=PR.base(st, "/World/M/d_motor", "Metals", "Aluminum_Anodized_Black", tile_m=0.1), prop=PR.omnipbr(st, "/World/M/d_prop", (0.02, 0.02, 0.02), 0.4),
         blur=PR.omnipbr(st, "/World/M/d_blur", (0.05, 0.05, 0.05), 0.6), lens=PR.omnipbr(st, "/World/M/d_lens", (0.01, 0.012, 0.02), 0.05, 0.6),
         orange=PR.omnipbr(st, "/World/M/d_or", (0.85, 0.32, 0.02), 0.4), red=PR.omnipbr(st, "/World/M/d_red", (1, 0.05, 0.05), 0.3, emissive=((1, 0.05, 0.03), 4000.0)),
         green=PR.omnipbr(st, "/World/M/d_grn", (0.05, 1, 0.1), 0.3, emissive=((0.05, 1, 0.1), 4000.0)), lidar=PR.omnipbr(st, "/World/M/d_lidar", (0.55, 0.56, 0.58), 0.3, 0.8))
drone = st.DefinePrim("/World/Drone", "Xform"); d_op = UsdGeom.Xformable(drone).AddTransformOp()
box("/World/Drone/Body", (0, 0, 0), (0.42, 0.22, 0.13), D["body"]); box("/World/Drone/Battery", (-0.02, 0, 0.1), (0.3, 0.16, 0.07), D["orange"])
box("/World/Drone/Nose", (0.23, 0, -0.01), (0.06, 0.18, 0.1), D["body"])
PROPS = []
for i, a in enumerate((45, 135, 225, 315)):
    r = math.radians(a); ex, ey = 0.36 * math.cos(r), 0.36 * math.sin(r)
    box(f"/World/Drone/Arm{i}", (ex / 2, ey / 2, 0.02), (0.36, 0.035, 0.03), D["arm"], rot_z=a)
    cyl(f"/World/Drone/Motor{i}", (ex, ey, 0.05), 0.035, 0.05, D["motor"])
    p = st.DefinePrim(f"/World/Drone/Prop{i}", "Xform"); pop = UsdGeom.Xformable(p).AddTransformOp(); PROPS.append((pop, (ex, ey, 0.085), 1 if i % 2 else -1))
    box(f"/World/Drone/Prop{i}/blade", (0, 0, 0), (0.42, 0.035, 0.006), D["prop"])
    cyl(f"/World/Drone/Prop{i}/disk", (0, 0, 0), 0.21, 0.002, D["blur"])                  # motion-blur disk
    cyl(f"/World/Drone/Nav{i}", (ex * 1.02, ey * 1.02, 0.0), 0.012, 0.02, D["red"] if ey < 0 else D["green"])
for sy in (-1, 1):                                                                  # landing skids
    box(f"/World/Drone/Leg{sy+1}a", (0.08, sy * 0.13, -0.13), (0.02, 0.02, 0.2), D["arm"]); box(f"/World/Drone/Leg{sy+1}b", (-0.08, sy * 0.13, -0.13), (0.02, 0.02, 0.2), D["arm"])
    box(f"/World/Drone/Skid{sy+1}", (0, sy * 0.13, -0.235), (0.34, 0.025, 0.02), D["arm"])
cyl("/World/Drone/LidarMount", (0.0, 0, 0.16), 0.045, 0.04, D["body"]); cyl("/World/Drone/LidarHousing", (0.0, 0, 0.215), 0.043, 0.07, D["lidar"])
cyl("/World/Drone/GNSS", (-0.15, 0, 0.17), 0.03, 0.02, D["body"])
# gimbal: yoke + camera ball under the nose; the camera prim follows the gimbal pitch
gim = st.DefinePrim("/World/Drone/Gimbal", "Xform"); gim_op = UsdGeom.Xformable(gim).AddTransformOp()
box("/World/Drone/Gimbal/Yoke", (0, 0, 0.03), (0.04, 0.14, 0.05), D["body"])
sph = UsdGeom.Sphere.Define(st, "/World/Drone/Gimbal/Ball"); sph.CreateRadiusAttr(0.055); bind(sph.GetPrim(), D["body"])
cyl("/World/Drone/Gimbal/Lens", (0.05, 0, 0), 0.028, 0.03, D["lens"], axis="X")
GCAM = "/World/Drone/Gimbal/Cam"
gcam = UsdGeom.Camera.Define(st, GCAM); gcam.CreateFocalLengthAttr(12.0); gcam.CreateHorizontalApertureAttr(20.955)
gcam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500))
UsdGeom.Xformable(gcam).AddTransformOp().Set(Gf.Matrix4d(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0.07, 0, 0, 1))   # look along gimbal +x
GIMBAL_AT = np.array([0.24, 0.0, -0.1])

# ---------------------------------------------------------------- sensors: ray-traced lidar (Ouster OS0-128) on top of the drone
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.rtx.nodes")
from isaacsim.sensors.experimental.rtx import Lidar
LID = Lidar.create("/World/Drone/Lidar", config="OS0", variant="OS0_REV7_128ch10hz512res", translations=[[0.0, 0.0, 0.26]])
LID_PATH = "/World/Drone/Lidar/sensor"

chase = UsdGeom.Camera.Define(st, "/World/Chase"); chase.CreateFocalLengthAttr(18.0); chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000))
c_op = UsdGeom.Xformable(chase).AddTransformOp()


def look(op, eye, tgt):
    m = Gf.Matrix4d(); m.SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, tgt)), Gf.Vec3d(0, 0, 1)); op.Set(m.GetInverse())


# ---------------------------------------------------------------- inspection route (shared with the autopilot, route.py)
sys.path.insert(0, H); import route as RT
WP = RT.flight_plan(); STANDOFF = RT.STANDOFF; SPEED = 2.0
assert RT.PIERS == PIERS and abs(RT.G_BOT - G_BOT) < 1e-6, "route.py and the world disagree on the bridge"


def gnss(pos):
    """Satellite visibility from the drone: the deck blocks the sky overhead -> fewer satellites, worse HDOP."""
    under = (abs(pos[1]) < W / 2 + 1.5) and (-HALF < pos[0] < HALF) and pos[2] < DECK_TOP
    edge = max(0.0, 1.0 - (W / 2 + 1.5 - abs(pos[1])) / 4.0) if under else 1.0
    sats = int(round(6 + 14 * edge)) if under else 20
    hdop = 0.7 if not under else round(0.7 + 6.0 * (1 - edge), 1)
    fix = "RTK FIXED" if sats >= 16 else ("3D (degraded)" if sats >= 9 else "LOST: lidar odometry")
    return sats, hdop, fix


# ---------------------------------------------------------------- run
import cv2
if STILL:
    PR.path_tracing(spp_per_frame=16, total_spp=256, max_bounces=6)
    os.makedirs(os.path.join(DATA, "stills"), exist_ok=True)
    rp = rep.create.render_product("/World/Chase", (1920, 1080)); a = rep.AnnotatorRegistry.get_annotator("LdrColor"); a.attach([rp])
    dpos = np.array([PIERS[1] + 4.0, -5.6, 2.2])
    d_op.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 160.0)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*dpos)))
    shots = {"wide": ((-75, -95, 26), (0, 0, 6)), "drone": (dpos + np.array([2.2, -2.4, 0.6]), dpos), "under": ((-30, -12, 2.5), (-5, 3, 8))}
    for name, (eye, tgt) in shots.items():
        look(c_op, eye, tgt)
        for _ in range(3): rep.orchestrator.step(rt_subframes=16)
        cv2.imwrite(os.path.join(DATA, "stills", f"{name}.png"), np.asarray(a.get_data())[:, :, 2::-1]); print("STILL", name, flush=True)
    app.close(); sys.exit(0)

rp_g = rep.create.render_product(GCAM, (960, 540))
a_rgb = rep.AnnotatorRegistry.get_annotator("LdrColor"); a_rgb.attach([rp_g])
a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); a_dep.attach([rp_g])
rp_l = rep.create.render_product(LID_PATH, [1, 1], name="lidar_rp", render_vars=["GenericModelOutput", "RtxSensorMetadata"])
a_pc = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator"); a_pc.attach([rp_l])
import omni.timeline; omni.timeline.get_timeline_interface().play()

OUT = os.path.join(DATA, "flight_" + time.strftime("%Y%m%d_%H%M%S")); os.makedirs(OUT, exist_ok=True)
MAP = []                                                                           # accumulated lidar map (world frame)
HUD = None
if not HEADLESS:
    import omni.ui as ui
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Chase"
    pv_cam, pv_dep, pv_map = ui.ByteImageProvider(), ui.ByteImageProvider(), ui.ByteImageProvider()
    win = ui.Window("Inspection drone: sensor stack", width=1000, height=1000)
    with win.frame:
        with ui.VStack(spacing=4):
            ui.Label("Gimbal RGB (defect imagery)", height=18)
            ui.ImageWithProvider(pv_cam, width=960, height=540)
            with ui.HStack(height=250):
                with ui.VStack():
                    ui.Label("Depth: standoff to structure", height=18); ui.ImageWithProvider(pv_dep, width=470, height=232)
                with ui.VStack():
                    ui.Label("Ouster OS0-128 lidar map (side view)", height=18); ui.ImageWithProvider(pv_map, width=470, height=232)
            HUD = ui.Label("", height=120, style={"font_size": 18, "color": 0xFFE8F0FF})


def show(prov, img):
    rgba = np.concatenate([img, np.full(img.shape[:2] + (1,), 255, np.uint8)], -1) if img.shape[2] == 3 else img
    prov.set_data_array(np.ascontiguousarray(rgba), [rgba.shape[1], rgba.shape[0]])


def map_image(pts, w=470, h=232):
    im = np.zeros((h, w, 3), np.uint8); im[:] = (14, 18, 26)
    if len(pts):
        P = np.asarray(pts); u = ((P[:, 0] + 65) / 130 * (w - 1)).astype(int); v = ((14 - P[:, 2]) / 20 * (h - 1)).astype(int)
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h); z = np.clip((P[ok, 2] + 4) / 16, 0, 1)
        im[v[ok], u[ok]] = np.stack([60 + 195 * z, 200 - 80 * z, 255 - 200 * z], -1).astype(np.uint8)
    return im


tel = open(os.path.join(OUT, "telemetry.csv"), "w"); tel.write("t,x,y,z,yaw,element,standoff_m,gnss_sats,hdop,fix,lidar_pts\n")
MAV = os.environ.get("MAV") == "1"
if MAV:
    import socket
    MSOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); MSOCK.bind(("127.0.0.1", 15000)); MSOCK.setblocking(False)
MSTATE, last_wp = {}, -1
seg, s_in, rec_i, t, frame = 0, 0.0, 0, 0.0, 0
dt = 1 / 30
for _ in range(30): app.update()
while app.is_running():
    if MAV:                                                                     # pose from the real autopilot (ArduCopter SITL)
        try:
            while True: MSTATE = json.loads(MSOCK.recv(65536))
        except BlockingIOError:
            pass
        if "pos" not in MSTATE:
            app.update(); continue
        pos = np.array(MSTATE["pos"]); wi = int(MSTATE.get("wp", 1)) - 1
        b_ = WP[min(max(wi, 0), len(WP) - 1)]; tgt = b_[1]
        yaw = math.radians(90.0) - MSTATE["yaw"]                                 # NED heading -> world (x east, ccw)
        R = (Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(MSTATE["roll"]))) *
             Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), -math.degrees(MSTATE["pitch"]))) *
             Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))))
        d_op.Set(R * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pos)))
        gyaw = math.atan2(tgt[1] - pos[1], tgt[0] - pos[0]) - yaw                # 3-axis gimbal: pan to the target
        if wi != last_wp:
            s_in, leg, last_wp = 1e9, 1.0, wi                                    # new mission item -> save an inspection frame
        else:
            s_in, leg = 0.0, 1.0
    else:
        a_, b_ = WP[seg % len(WP)], WP[(seg + 1) % len(WP)]
        leg = max(np.linalg.norm(b_[0] - a_[0]), 0.3)
        u = min(s_in / leg, 1.0); u = u * u * (3 - 2 * u)
        pos = a_[0] + (b_[0] - a_[0]) * u + np.array([0.03 * math.sin(2.1 * t), 0.03 * math.sin(1.7 * t), 0.02 * math.sin(2.9 * t)])   # hover wobble
        tgt = a_[1] + (b_[1] - a_[1]) * u
        yaw = math.atan2(tgt[1] - pos[1], tgt[0] - pos[0]); gyaw = 0.0
        d_op.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pos)))
    pitch = -math.degrees(math.atan2(tgt[2] - (pos[2] + GIMBAL_AT[2]), max(np.hypot(*(tgt - pos)[:2]), 0.1)))
    gim_op.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch)) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(gyaw)))
               * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*GIMBAL_AT)))
    for pop, c, sgn in PROPS:
        pop.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), (sgn * 2900 * t) % 360)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*c)))
    for car in CARS:                                                                # traffic
        car[3] += car[2] * car[4] * dt
        if car[3] > HALF + 15: car[3] = -HALF - 15
        if car[3] < -HALF - 15: car[3] = HALF + 15
        car[0].Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(car[3], car[1], DECK_TOP + 0.08)))
    back = pos - 6.5 * np.array([math.cos(yaw), math.sin(yaw), 0]) + np.array([1.5 * math.sin(yaw), -1.5 * math.cos(yaw), 1.6])
    look(c_op, back, pos + np.array([0, 0, -0.2]))
    if HEADLESS: rep.orchestrator.step(rt_subframes=1)
    else: app.update()
    t += dt; frame += 1
    if frame % 2 == 0: set_water_normal(frame // 2)                         # animate the ripples (15 fps)
    if not MAV: s_in += SPEED * dt
    # --- sensors
    pc = a_pc.get_data(); npts = 0
    if isinstance(pc, dict) and pc.get("data") is not None and len(pc["data"]):
        P = np.asarray(pc["data"], np.float32); npts = len(P)
        Rw = np.array(UsdGeom.Xformable(st.GetPrimAtPath(LID_PATH)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        Pw = P @ Rw[:3, :3] + Rw[3, :3]                                              # USD row-vector convention
        if frame % 3 == 0: MAP.extend(Pw[:: 12].tolist())
    dep = np.asarray(a_dep.get_data(), np.float32)
    standoff = float(np.nanmedian(np.where(np.isfinite(dep[220:320, 400:560]), dep[220:320, 400:560], np.nan))) if dep.size else float("nan")
    sats, hdop, fix = gnss(pos)
    if MAV and "sats" in MSTATE:                                                # the autopilot's own GPS report
        sats, hdop = MSTATE["sats"], MSTATE.get("hdop", 0.0)
        fix = {6: "RTK FIXED", 5: "RTK FLOAT", 4: "DGPS", 3: "3D FIX"}.get(MSTATE.get("fix", 0), "NO FIX") + ("  (degraded under deck)" if sats < 10 else "")
        fix += f"   |  ArduCopter {MSTATE.get('mode', '?')}  {'ARMED' if MSTATE.get('armed') else 'disarmed'}  item {MSTATE.get('wp', 0)}/{len(WP) + 1}"
    if frame % 3 == 0:
        tel.write(f"{t:.2f},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},{yaw:.3f},{b_[2]},{standoff:.2f},{sats},{hdop},{fix},{npts}\n")
    if not HEADLESS and frame % 3 == 0:
        rgb = np.asarray(a_rgb.get_data())
        if rgb.size: show(pv_cam, rgb[:, :, :3])
        if dep.size:
            dv = np.nan_to_num(np.clip(dep / 12.0, 0, 1), nan=1.0); show(pv_dep, cv2.resize(cv2.applyColorMap((255 * (1 - dv)).astype(np.uint8), cv2.COLORMAP_TURBO)[:, :, ::-1], (470, 232)))
        if frame % 15 == 0: show(pv_map, map_image(MAP[-400000:]))
        HUD.text = (f"Element: {b_[2]:8s}   Standoff (depth): {standoff:4.1f} m  (target {STANDOFF:.1f} m)\n"
                    f"GNSS: {sats:2d} sats  HDOP {hdop:3.1f}  {fix}\n"
                    f"Lidar: {npts:6d} pts/frame   map {len(MAP):,} pts   Gimbal pitch {pitch:5.1f} deg\n"
                    f"Pos ({pos[0]:6.1f}, {pos[1]:5.1f}, {pos[2]:4.1f}) m   t {t:6.1f} s")
    if s_in >= leg:                                                                 # waypoint reached: save an inspection frame
        s_in = 0.0; seg += 1
        rgb = np.asarray(a_rgb.get_data())
        if rgb.size and seg <= len(WP):
            cv2.imwrite(os.path.join(OUT, f"{rec_i:04d}.png"), rgb[:, :, 2::-1])
            json.dump(dict(i=rec_i, element=b_[2], drone=[float(v) for v in pos], target=[float(v) for v in tgt], standoff_m=standoff,
                           gnss=dict(sats=sats, hdop=hdop, fix=fix)), open(os.path.join(OUT, f"{rec_i:04d}.json"), "w")); rec_i += 1
        if seg == len(WP) and MAP:
            np.save(os.path.join(OUT, "lidar_map.npy"), np.asarray(MAP, np.float32)); print("LAP", OUT, rec_i, len(MAP), flush=True)
            if HEADLESS: break
    if HEADLESS and frame > int(os.environ.get("FRAMES", "100000")): break
tel.close(); app.close()
