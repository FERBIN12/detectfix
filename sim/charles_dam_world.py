"""Charles River Dam crossing (Route 28 by the Museum of Science, Boston), inspected by a drone flown by real ArduCopter.

    OMNI_KIT_ACCEPT_EULA=YES ~/IsaacLab/env_isaaclab/bin/python charles_dam_world.py              # live (real-time RTX)
    HEADLESS=1 ~/IsaacLab/env_isaaclab/bin/python charles_dam_world.py                             # record one flight
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
import photoreal as PR                                                   # (sim/ is on sys.path as the script dir)

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
    water=PR.mdl_material(st, "/World/M/water", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "mdl", "HarbourWater.mdl"), "HarbourWater",
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


# ---------------------------------------------------------------- Craigie Drawbridge + dam lock canal: frame + materials
sys.path.insert(0, H); import route_dam as RT
cw, HC = RT.cw, RT.HEADING_C
M.update(
    gran=[PR.base(st, f"/World/M/gran{i}", "Stone", n, tile_m=t, extra={"diffuse_tint": ("c3", tint)}) for i, (n, t, tint) in enumerate((
          ("Granite_Light", 1.2, (0.62, 0.62, 0.60)), ("Granite_Dark", 1.4, (1.0, 1.0, 1.0)), ("Granite_Light", 0.9, (0.48, 0.47, 0.44)), ("Granite_Dark", 1.1, (1.3, 1.25, 1.15))))],
    mortar=PR.omnipbr(st, "/World/M/mortar", (0.06, 0.06, 0.055), 0.95),
    coping=PR.base(st, "/World/M/coping", "Stone", "Granite_Light", tile_m=2.0),
    tower=PR.base(st, "/World/M/tower", "Masonry", "Stucco", tile_m=2.5, extra={"diffuse_tint": ("c3", (0.80, 0.70, 0.55))}),
    trim=PR.base(st, "/World/M/trim", "Stone", "Granite_Light", tile_m=1.0, extra={"diffuse_tint": ("c3", (0.85, 0.78, 0.66))}),
    green=PR.omnipbr(st, "/World/M/dcrgreen", (0.06, 0.20, 0.13), 0.45, 0.5),          # DCR bridge green
    copper=PR.omnipbr(st, "/World/M/copper", (0.20, 0.42, 0.36), 0.55, 0.3),           # weathered copper lantern
    brickbeige=PR.base(st, "/World/M/brickbeige", "Masonry", "Adobe_Brick", tile_m=2.0),
    brickred=PR.base(st, "/World/M/brickred", "Masonry", "Brick_Wall_Red", tile_m=3.0),
    roof=PR.omnipbr(st, "/World/M/roof", (0.42, 0.43, 0.44), 0.6),
    glass=PR.omnipbr(st, "/World/M/glass", (0.06, 0.08, 0.10), 0.05, 0.6),
    iron=PR.omnipbr(st, "/World/M/iron", (0.03, 0.03, 0.03), 0.5, 0.6),
    lawn=PR.base(st, "/World/M/lawn", "Natural", "Grass_Cut", tile_m=3.0),
    pave=PR.base(st, "/World/M/pave", "Masonry", "Brick_Pavers", tile_m=1.5),
    museum=PR.base(st, "/World/M/museum", "Masonry", "Concrete_Smooth", tile_m=4.0),
    white=PR.omnipbr(st, "/World/M/white", (0.9, 0.9, 0.88), 0.35),
    cable=PR.omnipbr(st, "/World/M/cable", (0.85, 0.85, 0.85), 0.3, 0.8),
    concrete=PR.base(st, "/World/M/concrete", "Masonry", "Concrete_Formed", tile_m=3.0),
    wet=PR.omnipbr(st, "/World/M/wet", (0.05, 0.055, 0.04), 0.3),
)


TEX = os.path.join(H, "..", "assets", "tex")


def scan(name, tile_m, tint=(1.0, 1.0, 1.0), metal=0.0, bump=1.0):
    """OmniPBR with a CC0 Poly Haven scan (diffuse + normal + roughness), world-projected so boxes need no UVs."""
    path = f"/World/M/scan_{name}_{int(tile_m * 100)}"
    m = PR.mdl_material(st, path, "OmniPBR.mdl", "OmniPBR", {"project_uvw": True, "world_or_object": True,
                        "texture_scale": ("f2", (1.0 / tile_m, 1.0 / tile_m)), "metallic_constant": float(metal),
                        "diffuse_tint": ("c3", tint), "reflection_roughness_texture_influence": 1.0, "bump_factor": float(bump)})
    sh = UsdShade.Shader.Get(st, path + "/Shader")
    for k, f, cs in (("diffuse_texture", "diff", "sRGB"), ("normalmap_texture", "nor", "raw"), ("reflectionroughness_texture", "rough", "raw")):
        x = sh.CreateInput(k, Sdf.ValueTypeNames.Asset); x.Set(Sdf.AssetPath(os.path.join(TEX, f"{name}_{f}.jpg"))); x.GetAttr().SetColorSpace(cs)
    return m


M.update(wallscan=scan("large_sandstone_blocks", 3.2, (0.78, 0.78, 0.76)), towerscan=scan("white_sandstone_blocks_02", 2.2, (0.95, 0.88, 0.76)),
         steelscan=scan("green_metal_rust", 1.6, (0.85, 1.0, 0.9), metal=0.5), asphaltscan=scan("asphalt_02", 4.0),
         pavescan=scan("concrete_pavement", 2.5), brickscan=scan("brick_wall_02", 2.5, (1.0, 0.92, 0.8)), rustscan=scan("rusty_metal_02", 1.0, metal=0.6),
         wetscan=scan("large_sandstone_blocks", 3.2, (0.26, 0.28, 0.21), bump=1.4))


def cbox(path, c, r, z, sc, sr, sz, m, dyaw=0.0):
    """box in the canal frame (c along the canal, r across)."""
    return box(path, cw(c, r, z), (sc, sr, sz), m, rot_z=HC + dyaw)


def ground_z(x, y):
    return np.zeros_like(np.asarray(x, float))


# ---------------------------------------------------------------- water (Harbour USV turbid water), canal + basin
RN = (160, 160); rx = np.linspace(-500, 500, RN[0]); ry = np.linspace(-500, 500, RN[1]); RX, RY = np.meshgrid(rx, ry)
RF = [[j * RN[0] + i, j * RN[0] + i + 1, (j + 1) * RN[0] + i + 1, (j + 1) * RN[0] + i] for j in range(RN[1] - 1) for i in range(RN[0] - 1)]
_rv = mesh("/World/River", np.stack([RX, RY, 0.02 * np.sin(0.05 * RX + 0.03 * RY)], -1).reshape(-1, 3), RF, M["water"])
_rv.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.tile([0, 0, 1.0], (RN[0] * RN[1], 1)).astype(np.float32))); _rv.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
WTILE, WTEX = 3.0, os.path.join(H, "..", "assets", "water")
UsdGeom.PrimvarsAPI(_rv.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
    Vt.Vec2fArray.FromNumpy((np.stack([RX, RY], -1).reshape(-1, 2) / WTILE).astype(np.float32)))
_wn = UsdShade.Shader.Get(st, "/World/M/water/Shader").CreateInput("normal_map_texture", Sdf.ValueTypeNames.Asset)
def set_water_normal(f):
    _wn.Set(Sdf.AssetPath(os.path.join(WTEX, f"water_n{f % 96:02d}.png"))); _wn.GetAttr().SetColorSpace("raw")
set_water_normal(0)
box("/World/Bed", (0, 0, -5.0), (1000, 1000, 0.2), M["soil"])

# ---------------------------------------------------------------- canal walls: individual granite blocks (ashlar), coping, wet band
WALL_B = -4.0
BLK_L, BLK_H = 1.6, 0.62
inst_protos = []
for i, m in enumerate(M["gran"]):
    c = UsdGeom.Cube.Define(st, f"/World/Blocks/Proto{i}"); c.CreateSizeAttr(1.0); bind(c.GetPrim(), m)
    inst_protos.append(c.GetPath())
UsdGeom.Imageable(st.GetPrimAtPath("/World/Blocks")).MakeInvisible() if False else None
recs = []
def wall_blocks(c0, c1, r_face, side):
    """courses of granite blocks along the canal; face at r_face, facing the water (side = -1 for the east wall)."""
    for k, z in enumerate(np.arange(WALL_B, RT.WALL_TOP - BLK_H / 2, BLK_H)):
        off = (k % 2) * BLK_L / 2
        for c in np.arange(c0 + off, c1, BLK_L):
            ln = min(BLK_L - 0.02, c1 - c)
            if ln < 0.2: continue
            jitter = rng.normal(0, 0.012)
            p = cw(c + ln / 2, r_face - side * (0.6 + jitter), z + BLK_H / 2)
            p = p + np.array([*(RT.R * side * rng.uniform(0.0, 0.03)), 0.0])               # blocks sit slightly proud/recessed
            recs.append((int(rng.choice(4, p=[0.4, 0.2, 0.25, 0.15])), p, (ln - 0.03, 1.2, BLK_H - 0.045)))
pass                                                                         # (scanned-block wall faces below replace per-block geometry)
if recs:
  pi = UsdGeom.PointInstancer.Define(st, "/World/Blocks/Wall")
  pi.CreatePrototypesRel().SetTargets(inst_protos)
  pi.CreateProtoIndicesAttr(Vt.IntArray([r_[0] for r_ in recs]))
  pi.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(np.array([r_[1] for r_ in recs], np.float32)))
  qz = Gf.Quath(Gf.Rotation(Gf.Vec3d(0, 0, 1), HC).GetQuat())
  pi.CreateOrientationsAttr(Vt.QuathArray([qz] * len(recs)))
  pi.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(np.array([r_[2] for r_ in recs], np.float32)))
for i_ in range(4): UsdGeom.Imageable(st.GetPrimAtPath(f"/World/Blocks/Proto{i_}")).MakeInvisible()
print("granite blocks", len(recs), flush=True)
for side, r_face in ((-1, RT.CANAL_W / 2), (1, -RT.CANAL_W / 2)):
    for nm, c0, c1 in (("Up", RT.CANAL_C0, -RT.DECK_W / 2 - 3), ("Dn", RT.DECK_W / 2 + 3, RT.CANAL_C1)):
        cc, lc = (c0 + c1) / 2, c1 - c0
        cbox(f"/World/Canal/Coping{nm}{side+1}", cc, r_face - side * 0.72, RT.WALL_TOP + 0.15, lc, 1.56, 0.3, M["pavescan"])
        cbox(f"/World/Canal/CopingLip{nm}{side+1}", cc, r_face + side * 0.04, RT.WALL_TOP - 0.02, lc, 0.1, 0.08, M["pavescan"])       # overhanging lip
        cbox(f"/World/Canal/Wet{nm}{side+1}", cc, r_face + side * 0.005, 0.25, lc, 0.02, 1.1, M["wetscan"])     # algae/wet band
        cbox(f"/World/Canal/Wall{nm}{side+1}", cc, r_face - side * 0.6, (WALL_B + RT.WALL_TOP) / 2, lc, 1.2, RT.WALL_TOP - WALL_B, M["wallscan"])   # scanned granite-block face
        cbox(f"/World/Canal/Bank{nm}{side+1}", cc, r_face - side * 21.2, RT.WALL_TOP / 2 - 0.3, lc, 40.0, RT.WALL_TOP + 0.6, M["gran"][2])
        cbox(f"/World/Canal/Walkway{nm}{side+1}", cc, r_face - side * 3.0, RT.WALL_TOP + 0.31, lc, 3.0, 0.05, M["pavescan"])
        cbox(f"/World/Canal/Grass{nm}{side+1}", cc, r_face - side * 22.7, RT.WALL_TOP + 0.31, lc, 36.4, 0.04, M["lawn"])
        for k, c in enumerate(np.arange(c0 + 4, c1 - 2, 9.0)):                                     # iron bollards on the coping
            p = cw(c, r_face - side * 0.45, RT.WALL_TOP + 0.6); cyl(f"/World/Canal/Bollard{nm}{side+1}_{k}", tuple(p), 0.17, 0.6, M["iron"])
# trees lining both canal banks (fall)
SPECIES = ["Red_Maple", "Black_Oak_Fall", "Scarlet_Oak_fall", "Gray_Birch_fall", "Honey_Locust_Fall", "American_Beech", "Red_Oak"]
k = 0
for side in (-1, 1):
    for c in np.arange(RT.CANAL_C0 + 3, RT.CANAL_C1, 7.5):
        if abs(c) < RT.DECK_W / 2 + 10: continue
        p = cw(c + rng.uniform(-1.5, 1.5), -side * (RT.CANAL_W / 2 + rng.uniform(3.5, 6.5)))
        t = st.DefinePrim(f"/World/Trees/T{k}", "Xform"); UsdGeom.Xformable(t).AddTranslateOp().Set(Gf.Vec3d(0, 0, RT.WALL_TOP + 0.3))
        PR.tree(st, f"/World/Trees/T{k}/tree", SPECIES[int(rng.integers(len(SPECIES)))], (float(p[0]), float(p[1])), float(rng.uniform(0, 360)), float(rng.uniform(0.8, 1.15))); k += 1

# ---------------------------------------------------------------- inspection support boat (Zodiac RIB, CC-BY, from the Harbour USV project) + landing pad
boat = st.DefinePrim("/World/Boat", "Xform"); bx_ = UsdGeom.Xformable(boat)
bx_.AddTranslateOp().Set(Gf.Vec3d(float(RT.LAUNCH[0] - 1.2 * RT.C[0]), float(RT.LAUNCH[1] - 1.2 * RT.C[1]), 0.40))
bx_.AddRotateZOp().Set(float(HC + 90.0)); bx_.AddRotateXOp().Set(90.0)                     # asset is Y-up, 6.6 m long along its Z
bb = st.DefinePrim("/World/Boat/rib", "Xform"); bb.GetReferences().AddReference(os.path.join(H, "..", "assets", "rib", "rib.usd"), "/World")
M["pad"] = PR.omnipbr(st, "/World/M/pad_orange", (0.85, 0.35, 0.03), 0.6)
cbox("/World/BoatPad/Pad", RT.LAUNCH[0] * 0 + float(np.dot(RT.LAUNCH[:2], RT.C)), float(np.dot(RT.LAUNCH[:2], RT.R)), RT.LAUNCH[2] - 0.04, 1.1, 1.1, 0.06, M["pad"])
cbox("/World/BoatPad/PadH1", float(np.dot(RT.LAUNCH[:2], RT.C)), float(np.dot(RT.LAUNCH[:2], RT.R)) - 0.2, RT.LAUNCH[2] - 0.005, 0.5, 0.07, 0.01, M["white"])
cbox("/World/BoatPad/PadH2", float(np.dot(RT.LAUNCH[:2], RT.C)), float(np.dot(RT.LAUNCH[:2], RT.R)) + 0.2, RT.LAUNCH[2] - 0.005, 0.5, 0.07, 0.01, M["white"])
cbox("/World/BoatPad/PadH3", float(np.dot(RT.LAUNCH[:2], RT.C)), float(np.dot(RT.LAUNCH[:2], RT.R)), RT.LAUNCH[2] - 0.005, 0.07, 0.4, 0.01, M["white"])

# ---------------------------------------------------------------- the Craigie drawbridge (steel bascule, DCR green) on granite abutment piers
for side in (-1, 1):
    r0 = side * (RT.CANAL_W / 2 + 4.0)
    cbox(f"/World/Craigie/Pier{side+1}", 0, r0, (WALL_B + RT.WALL_TOP + 0.3) / 2, RT.DECK_W + 6, 8.0, RT.WALL_TOP + 0.3 - WALL_B, M["gran"][0])
    cbox(f"/World/Craigie/PierWet{side+1}", 0, side * (RT.CANAL_W / 2 - 0.005), 0.25, RT.DECK_W + 6, 0.02, 1.1, M["wetscan"])
RIV = []                                                                     # rivet positions (instanced)
for g, gc in enumerate(np.linspace(-RT.DECK_W / 2 + 0.8, RT.DECK_W / 2 - 0.8, 8)):                # riveted plate girders across the canal
    L_ = RT.SPAN + 3
    cbox(f"/World/Craigie/G{g}/Web", gc, 0, RT.CLEAR + 0.6, 0.03, L_, 1.1, M["steelscan"])
    cbox(f"/World/Craigie/G{g}/TopFl", gc, 0, RT.CLEAR + 1.17, 0.42, L_, 0.04, M["steelscan"])
    cbox(f"/World/Craigie/G{g}/BotFl", gc, 0, RT.CLEAR + 0.03, 0.42, L_, 0.04, M["steelscan"])
    for k_, rr in enumerate(np.arange(-L_ / 2 + 0.6, L_ / 2, 1.3)):                                  # web stiffeners (both sides)
        for sd in (-1, 1):
            cbox(f"/World/Craigie/G{g}/Stf{k_}_{sd+1}", gc + sd * 0.07, rr, RT.CLEAR + 0.6, 0.11, 0.025, 1.1, M["steelscan"])
    for rr in np.arange(-L_ / 2 + 0.1, L_ / 2, 0.11):                                               # rivet rows on the bottom flange
        for dc in (-0.15, 0.15):
            RIV.append(cw(gc + dc, rr, RT.CLEAR + 0.01))
rv = UsdGeom.Sphere.Define(st, "/World/Craigie/RivetProto"); rv.CreateRadiusAttr(0.011); bind(rv.GetPrim(), M["steelscan"])
ri = UsdGeom.PointInstancer.Define(st, "/World/Craigie/Rivets"); ri.CreatePrototypesRel().SetTargets([rv.GetPath()])
ri.CreateProtoIndicesAttr(Vt.IntArray([0] * len(RIV))); ri.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(np.array(RIV, np.float32)))
UsdGeom.Imageable(rv.GetPrim()).MakeInvisible()
for k, rr in enumerate(np.linspace(-RT.SPAN / 2, RT.SPAN / 2, 7)):                                   # floor beams
    cbox(f"/World/Craigie/Floorbeam{k}", 0, rr, RT.CLEAR + 0.95, RT.DECK_W - 1, 0.3, 0.42, M["steelscan"])
cbox("/World/Craigie/Deck", 0, 0, RT.DECK_TOP - 0.15, RT.DECK_W, RT.SPAN + 12, 0.3, M["deck"])
cbox("/World/Craigie/Road", 0, 0, RT.DECK_TOP + 0.03, RT.DECK_W - 6, RT.SPAN + 12, 0.08, M["asphaltscan"])
for side in (-1, 1):                                                         # fascia girders + arched lattice railing panels
    cc = side * (RT.DECK_W / 2 - 0.2)
    cbox(f"/World/Craigie/Fascia{side+1}", cc, 0, RT.DECK_TOP - 0.6, 0.35, RT.SPAN + 12, 1.2, M["steelscan"])
    cbox(f"/World/Craigie/RailTop{side+1}", cc, 0, RT.DECK_TOP + 1.15, 0.12, RT.SPAN + 12, 0.1, M["green"])
    cbox(f"/World/Craigie/RailBot{side+1}", cc, 0, RT.DECK_TOP + 0.15, 0.12, RT.SPAN + 12, 0.1, M["green"])
    for j, rr in enumerate(np.arange(-RT.SPAN / 2 - 6, RT.SPAN / 2 + 6.01, 2.0)):
        cbox(f"/World/Craigie/Post{side+1}_{j}", cc, rr, RT.DECK_TOP + 0.65, 0.12, 0.12, 1.0, M["green"])
        for a in np.linspace(0.15, 2.99, 7):                                                     # small arch in each panel
            cbox(f"/World/Craigie/Arc{side+1}_{j}_{int(a*100)}", cc, rr + 1.0 - math.cos(a) * 0.9, RT.DECK_TOP + 0.25 + math.sin(a) * 0.7, 0.05, 0.28, 0.05, M["green"], dyaw=0)
    cbox(f"/World/Craigie/Walk{side+1}", side * (RT.DECK_W / 2 - 2.0), 0, RT.DECK_TOP + 0.12, 3.0, RT.SPAN + 12, 0.2, M["pavescan"])

# ---------------------------------------------------------------- the two Beaux-Arts gate towers at the upstream corners of the bridge
def gate_tower(path, c, r, h, lantern):
    cbox(path + "/Base", c, r, RT.WALL_TOP + 1.0, 6.2, 6.2, 2.0, M["trim"])
    cbox(path + "/Shaft", c, r, RT.WALL_TOP + 2 + h / 2, 5.2, 5.2, h, M["towerscan"])
    for q, zq in enumerate(np.arange(RT.WALL_TOP + 2.6, RT.WALL_TOP + 2 + h * 0.35, 0.7)):         # rusticated base courses
        cbox(path + f"/Rust{q}", c, r, zq, 5.32, 5.32, 0.5, M["towerscan"])
    for qc, qr in ((2.55, 2.55), (2.55, -2.55), (-2.55, 2.55), (-2.55, -2.55)):                      # corner quoins
        cbox(path + f"/Quoin{int(qc > 0)}{int(qr > 0)}", c + qc, r + qr, RT.WALL_TOP + 2 + h / 2, 0.35, 0.35, h, M["trim"])
    cbox(path + "/Cornice", c, r, RT.WALL_TOP + 2 + h + 0.3, 6.0, 6.0, 0.6, M["trim"])
    for fi, fz in enumerate((0.45, 0.75)):                                                    # windows on each face
        for wi, (dc, dr, yaw) in enumerate(((2.61, 0, 0), (-2.61, 0, 0), (0, 2.61, 90), (0, -2.61, 90))):
            cbox(path + f"/Win{fi}_{wi}", c + dc, r + dr, RT.WALL_TOP + 2 + h * fz, 0.05, 1.4, 2.2, M["glass"], dyaw=yaw)
            cbox(path + f"/WinHead{fi}_{wi}", c + dc * 1.01, r + dr * 1.01, RT.WALL_TOP + 2 + h * fz + 1.25, 0.12, 1.9, 0.3, M["trim"], dyaw=yaw)
            cbox(path + f"/WinSill{fi}_{wi}", c + dc * 1.02, r + dr * 1.02, RT.WALL_TOP + 2 + h * fz - 1.2, 0.18, 1.7, 0.14, M["trim"], dyaw=yaw)
    if lantern:                                                                               # operator's lantern with green copper roof
        cbox(path + "/Lantern", c, r, RT.WALL_TOP + 2 + h + 1.8, 4.2, 4.2, 2.4, M["glass"])
        cbox(path + "/LanternFrame", c, r, RT.WALL_TOP + 2 + h + 1.8, 4.4, 4.4, 0.25, M["copper"])
        cbox(path + "/LanternRoof", c, r, RT.WALL_TOP + 2 + h + 3.3, 4.8, 4.8, 0.6, M["copper"])
        cbox(path + "/LanternCap", c, r, RT.WALL_TOP + 2 + h + 3.9, 2.4, 2.4, 0.6, M["copper"])
    else:
        cbox(path + "/Parapet", c, r, RT.WALL_TOP + 2 + h + 0.9, 5.4, 5.4, 0.6, M["trim"])
gate_tower("/World/TowerW", -RT.DECK_W / 2 - 4.0, -(RT.CANAL_W / 2 + 4.5), 12.5, False)
gate_tower("/World/TowerE", -RT.DECK_W / 2 - 4.0, RT.CANAL_W / 2 + 4.5, 14.0, True)
# beige brick lock gatehouse with a hip roof (east bank, upstream)
GH = (-RT.DECK_W / 2 - 22.0, RT.CANAL_W / 2 + 14.0)
cbox("/World/Gatehouse/Body", GH[0], GH[1], RT.WALL_TOP + 3.5, 18.0, 12.0, 7.0, M["brickscan"])
for k in range(6):                                                                            # stepped hip roof
    cbox(f"/World/Gatehouse/Roof{k}", GH[0], GH[1], RT.WALL_TOP + 7.2 + k * 0.55, 18.6 - k * 2.6, 12.6 - k * 2.0, 0.55, M["roof"])
for k, dc in enumerate(np.linspace(-6, 6, 4)):
    cbox(f"/World/Gatehouse/Win{k}", GH[0] + dc, GH[1] - 6.02, RT.WALL_TOP + 3.8, 1.6, 0.05, 3.0, M["glass"])
# cobra-head street light on the coping (as in the photo)
p = cw(-RT.DECK_W / 2 - 9, RT.CANAL_W / 2 + 2.0); cyl("/World/Canal/StreetPole", (p[0], p[1], RT.WALL_TOP + 5), 0.12, 10.0, M["steel"])

# ---------------------------------------------------------------- Lechmere Viaduct right behind the bridge: concrete arches + green steel arch over the canal
VC = RT.DECK_W / 2 + 18.0
for k, rr in enumerate(np.arange(-160, 161, 16.0)):
    if abs(rr) < RT.CANAL_W / 2 + 3: continue
    cbox(f"/World/Viaduct/Pier{k}", VC, rr, (WALL_B + 9.0) / 2, 9.0, 2.2, 9.0 - WALL_B, M["concrete"])
cbox("/World/Viaduct/Deck", VC, 0, 9.6, 9.5, 320, 1.2, M["concrete"])
for side in (-1, 1):
    cbox(f"/World/Viaduct/Parapet{side+1}", VC + side * 4.5, 0, 10.7, 0.3, 320, 1.0, M["concrete"])
for side in (-1, 1):                                                         # green steel arch truss over the canal span
    for a in np.linspace(0.05, math.pi - 0.05, 22):
        rr = -14.0 * math.cos(a); zz = 10.2 + 7.0 * math.sin(a)
        cbox(f"/World/Viaduct/ArchChord{side+1}_{int(a*100)}", VC + side * 4.2, rr, zz, 0.45, 2.2, 0.45, M["green"], dyaw=0)
    for k, rr in enumerate(np.linspace(-12, 12, 9)):
        zz = 10.2 + 7.0 * math.sqrt(max(0.0, 1 - (rr / 14.0) ** 2))
        cbox(f"/World/Viaduct/Hanger{side+1}_{k}", VC + side * 4.2, rr, (10.2 + zz) / 2, 0.15, 0.15, zz - 10.2, M["green"])
for k, rr in enumerate(np.linspace(-11, 11, 6)):
    cbox(f"/World/Viaduct/Brace{k}", VC, rr, 10.2 + 7.0 * math.sqrt(max(0.0, 1 - (rr / 14.0) ** 2)), 8.6, 0.2, 0.2, M["green"])

# ---------------------------------------------------------------- Museum of Science (brick, clock tower, planetarium dome) and the Zakim beyond
MS = (-RT.DECK_W / 2 - 70.0, -60.0)                                          # on the dam, upstream-west of the canal
cbox("/World/Museum/Main", MS[0], MS[1], 11.0, 90, 50, 22, M["brickred"])
cbox("/World/Museum/Glass", MS[0] + 45.1, MS[1], 12.0, 0.3, 46, 16, M["glass"])
cbox("/World/Museum/Clock", MS[0] + 20, MS[1] + 18, 22.0, 9, 9, 44, M["brickred"])
cbox("/World/Museum/ClockFace", MS[0] + 24.6, MS[1] + 18, 38.0, 0.1, 5, 5, M["white"])
cbox("/World/Museum/Spire", MS[0] + 20, MS[1] + 18, 47.0, 1.2, 1.2, 8, M["white"])
dome = UsdGeom.Sphere.Define(st, "/World/Museum/Planetarium"); dome.CreateRadiusAttr(11.0)
UsdGeom.Xformable(dome).AddTranslateOp().Set(Gf.Vec3d(*cw(MS[0] - 20, MS[1] - 10, 22.0))); bind(dome.GetPrim(), M["white"])
def tower(path, base_xy, h, spread, m):
    for sx in (-1, 1):
        a = np.array([base_xy[0] + sx * spread, base_xy[1], 0]); b = np.array([base_xy[0], base_xy[1], h * 0.62])
        mid = (a + b) / 2; ln = np.linalg.norm(b - a)
        c = UsdGeom.Cube.Define(st, f"{path}/Leg{sx+1}"); c.CreateSizeAttr(1.0); xf = UsdGeom.Xformable(c)
        xf.AddTranslateOp().Set(Gf.Vec3d(*mid)); xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*((b - a) / ln))).GetQuat()))
        xf.AddScaleOp().Set(Gf.Vec3f(3.0, 4.0, float(ln))); bind(c.GetPrim(), m)
    box(f"{path}/Spire", (base_xy[0], base_xy[1], h * 0.81), (3.0, 4.0, h * 0.38), m)
ZK = RT.geo_to_world(42.36860, -71.06180)
for t_, dy in (("TowerN", 60), ("TowerS", -60)):
    tower(f"/World/Zakim/{t_}", (ZK[0], ZK[1] + dy), 95.0, 14.0, M["white"])
box("/World/Zakim/Deck", (ZK[0], ZK[1], 28.0), (40.0, 420.0, 2.0), M["concrete"])
for t_, dy in (("N", 60), ("S", -60)):
    for k in range(10):
        for side in (-1, 1):
            a = np.array([ZK[0], ZK[1] + dy, 62 + 2.6 * k]); b = np.array([ZK[0] + side * 8, ZK[1] + dy + (k + 1) * 9.0 * (1 if dy > 0 else -1), 29.0])
            mid = (a + b) / 2; ln = np.linalg.norm(b - a)
            c = UsdGeom.Cylinder.Define(st, f"/World/Zakim/Cab{t_}{k}_{side+1}"); c.CreateRadiusAttr(0.12); c.CreateHeightAttr(float(ln))
            xf = UsdGeom.Xformable(c); xf.AddTranslateOp().Set(Gf.Vec3d(*mid))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*((b - a) / ln))).GetQuat())); bind(c.GetPrim(), M["cable"])
TD = RT.geo_to_world(42.36620, -71.06210); box("/World/City/TDGarden", (TD[0], TD[1], 18), (140, 110, 36), M["museum"])
SKY_M = [M["brickred"], PR.omnipbr(st, "/World/M/bld3", (0.10, 0.13, 0.16), 0.08, 0.6), PR.omnipbr(st, "/World/M/bld4", (0.55, 0.57, 0.6), 0.2, 0.5), M["museum"]]
DT = RT.geo_to_world(42.3570, -71.0600)
for i in range(60):
    c = DT + np.array([rng.normal(0, 260), rng.normal(0, 200), 0]); h = float(rng.uniform(40, 200))
    box(f"/World/City/Tower{i}", (c[0], c[1], h / 2), (rng.uniform(25, 50), rng.uniform(25, 50), h), SKY_M[1 if h > 80 else int(rng.integers(0, 4))])
for i in range(80):
    ang = rng.uniform(0, 6.283); r = rng.uniform(320, 520); c = np.array([r * math.cos(ang), r * math.sin(ang)]); h = float(rng.uniform(10, 30))
    box(f"/World/City/Low{i}", (c[0], c[1], h / 2), (rng.uniform(20, 45), rng.uniform(20, 45), h), SKY_M[int(rng.choice([0, 0, 3, 2]))], rot_z=math.degrees(ang))

# ---------------------------------------------------------------- defects (ground truth): canal walls and drawbridge steel
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


for (c, side, z, kind, sev, el) in ((-34.0, -1, 1.0, "spall", 3, "canal_wall_E"), (-26.0, -1, 2.3, "crack", 2, "canal_wall_E"), (-41.0, -1, 0.8, "efflorescence", 2, "canal_wall_E"),
                                     (22.0, -1, 1.6, "rust_streak", 2, "canal_wall_E"), (-22.0, 1, 1.2, "crack", 3, "canal_wall_W"), (-38.0, 1, 2.0, "spall", 2, "canal_wall_W"),
                                     (8.0, 1, 1.5, "efflorescence", 1, "canal_wall_W")):
    r_face = -side * RT.CANAL_W / 2 if False else (RT.CANAL_W / 2 if side == -1 else -RT.CANAL_W / 2)
    n = np.array([*(RT.R * (-1 if side == -1 else 1)), 0.0])
    defect(kind, el, cw(c, r_face, z), sev, n)
for (gc, rr, kind, sev) in ((-4.6, 2.0, "rust_streak", 2), (4.6, -3.0, "rust_streak", 1)):
    defect(kind, "bascule_girders", cw(gc + 0.26, rr, RT.CLEAR + 0.9), sev, np.array([*RT.C, 0.0]))
json.dump(dict(bridge="Craigie Drawbridge (NBI B160134EMDOTNBI) and the Charles River Dam lock canal walls, Boston", defects=DEFECTS),
          open(os.path.join(DATA, "defects_gt.json"), "w"), indent=1)

# ---------------------------------------------------------------- traffic on Route 28 over the drawbridge
CAR_PROTOS = PR.parked_car_protos(st)
CARS = []
HR = math.degrees(math.atan2(RT.R[1], RT.R[0]))
for i in range(10):
    lane = [-7.0, -4.0, -1.5, 1.5, 4.0, 7.0][i % 6]; dirn = 1 if lane > 0 else -1
    root = st.DefinePrim(f"/World/Traffic/Car{i}", "Xform"); op = UsdGeom.Xformable(root).AddTransformOp()
    PR.parked_car(st, f"/World/Traffic/Car{i}/c", CAR_PROTOS[i % len(CAR_PROTOS)], (0, 0), 0 if dirn > 0 else 180)
    CARS.append([op, lane, dirn, rng.uniform(-60, 60), rng.uniform(8, 13)])
def car_pose(u, lane):
    z = RT.DECK_TOP + 0.08 if abs(u) < RT.SPAN / 2 + 6 else RT.WALL_TOP + 0.35
    p = cw(lane, u, z)                                                      # lanes are offsets along the canal axis
    return Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), HR)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*p))
for c_ in CARS: c_[0].Set(car_pose(c_[3], c_[1]))
for nm, r0, r1 in (("RoadW", -110, -RT.SPAN / 2 - 6), ("RoadE", RT.SPAN / 2 + 6, 110)):          # Route 28 approaches
    p = cw(0, (r0 + r1) / 2, RT.WALL_TOP + 0.3); box(f"/World/Road/{nm}", p, (abs(r1 - r0), RT.DECK_W - 6, 0.1), M["asphaltscan"], rot_z=HR)
    for sd in (-1, 1):
        p2 = cw(sd * (RT.DECK_W / 2 - 1.5), (r0 + r1) / 2, RT.WALL_TOP + 0.38); box(f"/World/Road/{nm}Walk{sd+1}", p2, (abs(r1 - r0), 3.0, 0.16), M["pavescan"], rot_z=HR)
PR.sky(st, hdr="Cloudy/kloofendal_48d_partly_cloudy_4k.hdr", dome_intensity=1600.0, sun=(1400.0, (-55, 10, 200)))   # soft, overcast-ish like the reference photos

# ---------------------------------------------------------------- inspection drone: Holybro X500 V2 from PX4's meshes (x500.py)
from x500 import build_x500
drone = st.DefinePrim("/World/Drone", "Xform"); d_op = UsdGeom.Xformable(drone).AddTransformOp()
PROPS, gim_op, GCAM = build_x500(st, "/World/Drone/X500", PR)
GIMBAL_AT = np.array([0.025, 0.0, -0.17])                                   # pivot position in the drone frame (for aiming)

# ---------------------------------------------------------------- sensors: ray-traced lidar (Ouster OS0-128) on top of the drone
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.rtx.nodes")
from isaacsim.sensors.experimental.rtx import Lidar
LID = Lidar.create("/World/Drone/Lidar", config="OS0", variant="OS0_REV7_128ch10hz512res", translations=[[0.0, 0.0, 0.16]])
LID_PATH = "/World/Drone/Lidar/sensor"

chase = UsdGeom.Camera.Define(st, "/World/Chase"); chase.CreateFocalLengthAttr(18.0); chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000))
c_op = UsdGeom.Xformable(chase).AddTransformOp()


def look(op, eye, tgt):
    m = Gf.Matrix4d(); m.SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, tgt)), Gf.Vec3d(0, 0, 1)); op.Set(m.GetInverse())


# ---------------------------------------------------------------- inspection route (shared with the autopilot, route.py)
WP = RT.flight_plan(); STANDOFF = RT.STANDOFF; SPEED = 1.5


def gnss(pos):
    """Satellite visibility from the drone: the deck blocks the sky overhead -> fewer satellites, worse HDOP."""
    under = RT.under_deck(pos)
    cc_ = abs(float(np.dot(pos[:2], RT.C)))
    edge = max(0.0, 1.0 - (RT.DECK_W / 2 + 0.5 - cc_) / 4.0) if under else 1.0
    sats = int(round(6 + 14 * edge)) if under else 20
    hdop = 0.7 if not under else round(0.7 + 6.0 * (1 - edge), 1)
    fix = "RTK FIXED" if sats >= 16 else ("3D (degraded)" if sats >= 9 else "LOST: lidar odometry")
    return sats, hdop, fix


# ---------------------------------------------------------------- semantic classes for the simulated thermal camera (thermal.py)
from pxr import UsdSemantics
THERMAL_BY_PATH = (("/World/Drone", "drone"), ("/World/Boat", "boat"), ("/World/BoatPad", "boat"), ("/World/Traffic", "vehicle"), ("/World/Trees", "vegetation"),
                   ("/World/River", "water"), ("/World/Canal/Wet", "granite_wet"), ("/World/Craigie/PierWet", "granite_wet"))
THERMAL_BY_DEFECT = dict(efflorescence="defect_moisture", spall="defect_delamination", crack="defect_crack", rust_streak="steel_rust")
THERMAL_BY_MAT = (("water", "water"), ("wet", "granite_wet"), ("algae", "granite_wet"), ("gran", "granite"), ("wallscan", "granite"),
                  ("large_sandstone", "granite"), ("coping", "granite"), ("trim", "granite"), ("mortar", "granite"), ("pier", "granite"),
                  ("riprap", "granite"), ("asphalt", "asphalt"), ("rust", "steel_rust"), ("rebar", "steel_rust"), ("steel", "steel"),
                  ("green_metal", "steel"), ("iron", "steel"), ("dcrgreen", "steel"), ("cable", "steel"), ("copper", "steel"),
                  ("paint", "steel"), ("yellow", "steel"), ("glass", "water"), ("brick", "brick"), ("tower", "brick"),
                  ("white_sandstone", "brick"), ("pave", "concrete"), ("concrete", "concrete"), ("museum", "concrete"),
                  ("deck", "concrete"), ("girder", "concrete"), ("roof", "concrete"), ("white", "concrete"), ("grass", "vegetation"),
                  ("lawn", "vegetation"), ("soil", "vegetation"), ("pad", "boat"))


def label(prim, cls):
    UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr().Set([cls])


for d_ in DEFECTS:
    label(st.GetPrimAtPath(f"/World/Defects/{d_['id']}"), THERMAL_BY_DEFECT.get(d_["type"], "granite"))
for root_, cls in THERMAL_BY_PATH:
    if st.GetPrimAtPath(root_).IsValid(): label(st.GetPrimAtPath(root_), cls)
n_lab = 0
for prim in st.Traverse():
    ps = str(prim.GetPath())
    if not prim.IsA(UsdGeom.Gprim) or ps.startswith(("/World/Defects", "/World/Drone", "/World/Boat", "/World/BoatPad", "/World/Traffic", "/World/Trees")):
        continue
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    key = (mat.GetPath().name if mat else "").lower()
    cls = next((c for k, c in THERMAL_BY_PATH if ps.startswith(k)), None) or next((c for k, c in THERMAL_BY_MAT if k in key), "concrete")
    label(prim, cls); n_lab += 1
print("thermal labels:", n_lab, "prims", flush=True)

# ---------------------------------------------------------------- run
import cv2
if STILL:
    SPP = int(os.environ.get("SPP", "256"))                                    # SPP=64 for a quick preview
    PR.path_tracing(spp_per_frame=16, total_spp=SPP, max_bounces=6)
    os.makedirs(os.path.join(DATA, "stills"), exist_ok=True)
    rp = rep.create.render_product("/World/Chase", (1920, 1080)); a = rep.AnnotatorRegistry.get_annotator("LdrColor"); a.attach([rp])
    dpos = cw(-30.0, RT.CANAL_W / 2 - 2.5, 1.6)
    shots = {"canal": (cw(-78, 1.5, 2.2), cw(0, 0, 4.0)), "boat": (RT.LAUNCH + cw(-4.5, 3.0, 2.0) - cw(0, 0, 0), RT.LAUNCH), "drone": (dpos + cw(-2.6, -2.2, 0.5), dpos + cw(0.6, 0.8, 0)), "lights": (cw(-9.0, 0.5, 1.3), cw(-1.0, 3.2, 2.2)), "under": (cw(-16, -2.0, 1.4), cw(8, 2.0, 5.2)),
             "basin": (cw(-125, 45, 13), cw(-5, 0, 6))}
    look(c_op, *shots["canal"])
    for _ in range(40): rep.orchestrator.step(rt_subframes=1)                 # let textures stream in
    rep.orchestrator.step(rt_subframes=16); a.get_data()                       # the first capture comes back empty: throw it away
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    for p_ in ("/World/Drone", "/World/BoatPad", "/World/Boat"):
        r_ = bc.ComputeWorldBound(st.GetPrimAtPath(p_)).ComputeAlignedRange(); print("BBOX", p_, r_.GetMin(), r_.GetMax(), flush=True)
    for name, (eye, tgt) in shots.items():
        dp_ = {"drone": dpos, "lights": cw(-1.0, RT.CANAL_W / 2 - 3.0, 2.2)}.get(name, RT.LAUNCH)   # hovering at the wall / under the deck, else on the pad
        yaw_ = math.degrees(math.atan2(RT.R[1], RT.R[0])) if name in ("drone", "lights") else HC   # face the east wall
        d_op.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*dp_)))
        look(c_op, eye, tgt)
        for _ in range(3): rep.orchestrator.step(rt_subframes=16)
        cv2.imwrite(os.path.join(DATA, "stills", f"{name}.png"), np.asarray(a.get_data())[:, :, 2::-1]); print("STILL", name, flush=True)
    app.close(); sys.exit(0)

for k_, v_ in (("/rtx/post/histogram/enabled", True), ("/rtx/post/histogram/whiteScale", 6.0), ("/rtx/sceneDb/ambientLightIntensity", 1.6),
               ("/rtx/indirectDiffuse/enabled", True), ("/rtx/ambientOcclusion/enabled", True), ("/rtx/reflections/enabled", True)):
    try: carb.settings.get_settings().set(k_, v_)                         # live RTX: auto-exposure + ambient fill (canal walls sit in shade)
    except Exception: pass
rp_g = rep.create.render_product(GCAM, (960, 540))
a_rgb = rep.AnnotatorRegistry.get_annotator("LdrColor"); a_rgb.attach([rp_g])
a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); a_dep.attach([rp_g])
rp_l = rep.create.render_product(LID_PATH, [1, 1], name="lidar_rp", render_vars=["GenericModelOutput", "RtxSensorMetadata"])
a_pc = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator"); a_pc.attach([rp_l])
ROS = os.environ.get("ROS", "1") == "1"                                           # ROS 2 Jazzy topics for RViz2 (ros_pub.py)
if ROS:
    import thermal
    from ros_pub import RosPub
    a_sem = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": False}); a_sem.attach([rp_g])
    RP = RosPub()
    RP.mission(WP); RP.defects(DEFECTS)
    TRNG, last_pos = np.random.default_rng(7), None


def pose_of(path):
    """world position + quaternion (x, y, z, w) of a prim, and its 4x4 world matrix (USD row-vector convention)."""
    Mx = UsdGeom.Xformable(st.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    q = Mx.RemoveScaleShear().ExtractRotationQuat(); im = q.GetImaginary()
    return np.array(Mx.ExtractTranslation()), np.array([im[0], im[1], im[2], q.GetReal()]), Mx


def rel_pose(Mc, Mp):
    """pose of child matrix Mc expressed in parent matrix Mp."""
    R = Mc * Mp.GetInverse(); q = R.RemoveScaleShear().ExtractRotationQuat(); im = q.GetImaginary()
    return np.array(R.ExtractTranslation()), np.array([im[0], im[1], im[2], q.GetReal()])
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
PROF, PROF_T = os.environ.get("PROF") == "1", []
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
    pitch = max(-25.0, -math.degrees(math.atan2(tgt[2] - (pos[2] + GIMBAL_AT[2]), max(np.hypot(*(tgt - pos)[:2]), 0.1))))   # CGO3 tilts at most ~25 deg up
    gim_op.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch)) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(gyaw))))
    st.GetPrimAtPath("/World/Drone/X500/StrobeLight").GetAttribute("inputs:intensity").Set(60000.0 if (frame % 30) < 2 else 0.0)   # anti-collision strobe, 1 Hz
    for pop, c, sgn in PROPS:
        pop.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), (sgn * 2900 * t) % 360)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*c)))   # rotor spin
    for car in CARS:                                                                # traffic on Route 28
        car[3] += car[2] * car[4] * dt
        if car[3] > 80: car[3] = -80
        if car[3] < -80: car[3] = 80
        car[0].Set(car_pose(car[3], car[1]))
    back = pos - 6.5 * np.array([math.cos(yaw), math.sin(yaw), 0]) + np.array([1.5 * math.sin(yaw), -1.5 * math.cos(yaw), 1.6])
    look(c_op, back, pos + np.array([0, 0, -0.2]))
    _T0 = time.perf_counter()
    if HEADLESS and not MAV: rep.orchestrator.step(rt_subframes=1)              # recording: wait for every annotator
    else: app.update()                                                          # live: don't block on annotators
    _T1 = time.perf_counter()
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
    _T2 = time.perf_counter()
    if ROS:                                                                     # --- ROS 2: TF, odom, path, lidar, map, RGB, thermal
        bp, bq, Mb = pose_of("/World/Drone")
        vel = (bp - last_pos) / dt if last_pos is not None else np.zeros(3); last_pos = bp
        RP.pose(bp, bq, vel)
        for child, pth in (("os0_lidar", LID_PATH), ("gimbal_cam", GCAM)):
            RP.sensor_tf(child, *rel_pose(pose_of(pth)[2], Mb))
        if npts: RP.cloud(P)
        if npts and frame % 3 == 0: RP.map_extend(Pw[:: 12])
        if frame % 3 == 0:
            rgb = np.asarray(a_rgb.get_data())
            if rgb.size: RP.image("/camera/rgb", np.ascontiguousarray(rgb[:, :, :3]), "gimbal_cam")
            sem = a_sem.get_data()
            if isinstance(sem, dict) and np.asarray(sem.get("data", [])).size and dep.size:
                ids = np.asarray(sem["data"]).astype(np.int32); H_, W_ = ids.shape; cw_ = H_ * 5 // 4; x0 = (W_ - cw_) // 2   # Boson 5:4 crop
                id2c = {int(k): (v.get("class", "sky") if isinstance(v, dict) else str(v)) for k, v in sem["info"]["idToLabels"].items()}
                _, ib = thermal.synth(ids[:, x0:x0 + cw_], id2c, np.nan_to_num(dep[:, x0:x0 + cw_], nan=1e4, posinf=1e4), t, rng=TRNG)
                RP.image("/camera/thermal", ib, "thermal_cam")
        if frame % 15 == 0: RP.status(f"GNSS {sats} sats HDOP {hdop} {fix}")
        RP.spin()
    if PROF:
        PROF_T.append((_T1 - _T0, _T2 - _T1, time.perf_counter() - _T2))
        if len(PROF_T) == 90: print("PROF render/sensors/ros ms", np.round(1000 * np.mean(PROF_T, 0), 1), "max", np.round(1000 * np.max(PROF_T, 0), 1), flush=True); PROF_T.clear()
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
            if HEADLESS and not MAV: break
    if HEADLESS and frame > int(os.environ.get("FRAMES", "100000")): break
tel.close()
if ROS: RP.close()
app.close()
