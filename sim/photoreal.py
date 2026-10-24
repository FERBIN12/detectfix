"""Photoreal helpers: NVIDIA MDL materials (world-projected textures), HDRI sky + sun, path tracing,
real vegetation assets. Requires the Isaac asset root (NVIDIA S3 content), reached via get_assets_root_path().
"""
import carb
from pxr import UsdShade, Sdf, Gf, UsdLux, UsdGeom
from isaacsim.storage.native import get_assets_root_path

ROOT = get_assets_root_path()
BASE = f"{ROOT}/NVIDIA/Materials/Base"
SKY = f"{ROOT}/NVIDIA/Assets/Skies"
VEG = f"{ROOT}/NVIDIA/Assets/Vegetation"
VEG_UNIT = 0.01   # NVIDIA vegetation assets are authored in centimetres (metersPerUnit 0.01, Z-up)

_T = {bool: Sdf.ValueTypeNames.Bool, float: Sdf.ValueTypeNames.Float, int: Sdf.ValueTypeNames.Int}


def mdl_material(stage, path, url, sub, inputs=None):
    """Material prim bound to an MDL module/material; `inputs` override exposed parameters.
    Value types: bool/float/int, ('f2', (u, v)), ('c3', (r, g, b))."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(url, "mdl")
    sh.SetSourceAssetSubIdentifier(sub, "mdl")
    for k, v in (inputs or {}).items():
        if isinstance(v, tuple) and v[0] == "f2":
            sh.CreateInput(k, Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*v[1]))
        elif isinstance(v, tuple) and v[0] == "c3":
            sh.CreateInput(k, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*v[1]))
        else:
            sh.CreateInput(k, _T[type(v)]).Set(v)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    mat.CreateDisplacementOutput("mdl").ConnectToSource(out)
    mat.CreateVolumeOutput("mdl").ConnectToSource(out)
    return mat


def base(stage, path, category, name, tile_m=None, extra=None):
    """NVIDIA Base library material. tile_m: world-projected texture tile size in metres."""
    inputs = dict(extra or {})
    if tile_m:
        inputs.update({"project_uvw": True, "world_or_object": True, "texture_scale": ("f2", (1.0 / tile_m, 1.0 / tile_m))})
    return mdl_material(stage, path, f"{BASE}/{category}/{name}.mdl", name, inputs)


def omnipbr(stage, path, color, rough=0.4, metal=0.0, emissive=None):
    """Untextured OmniPBR (core library) for paints, metals, plastics, emitters."""
    inputs = {"diffuse_color_constant": ("c3", color), "reflection_roughness_constant": float(rough),
              "metallic_constant": float(metal)}
    if emissive:
        inputs.update({"enable_emission": True, "emissive_color": ("c3", emissive[0]), "emissive_intensity": float(emissive[1])})
    return mdl_material(stage, path, "OmniPBR.mdl", "OmniPBR", inputs)


def sky(stage, hdr="Cloudy/kloofendal_48d_partly_cloudy_4k.hdr", dome_intensity=1000.0, sun=(3500.0, (-38, 18, 128)), sun_color=(1.0, 0.95, 0.88)):
    dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
    dome.CreateIntensityAttr(dome_intensity)
    dome.CreateTextureFileAttr(f"{SKY}/{hdr}")
    dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
    s = UsdLux.DistantLight.Define(stage, "/World/Sun")
    s.CreateIntensityAttr(sun[0]); s.CreateAngleAttr(0.53); s.CreateColorAttr(Gf.Vec3f(*sun_color))
    UsdGeom.Xformable(s).AddRotateXYZOp().Set(Gf.Vec3f(*sun[1]))
    return dome, s


def path_tracing(spp_per_frame=8, total_spp=256, max_bounces=6, denoise=True):
    st = carb.settings.get_settings()
    st.set("/rtx/rendermode", "PathTracing")
    st.set("/rtx/pathtracing/spp", spp_per_frame)
    st.set("/rtx/pathtracing/totalSpp", total_spp)      # finite: a capture waits for this to converge (0 = forever = hang)
    st.set("/rtx/pathtracing/maxBounces", max_bounces)
    st.set("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 6)
    st.set("/rtx/pathtracing/optixDenoiser/enabled", denoise)
    st.set("/rtx/post/histogram/enabled", True)          # auto exposure
    st.set("/rtx/post/tonemap/op", 6)                    # ACES-style filmic curve
    st.set("/rtx/post/aa/op", 3)


def tree(stage, path, species, xy, yaw=0.0, scale=1.0):
    p = stage.DefinePrim(path, "Xform")
    p.GetReferences().AddReference(f"{VEG}/Trees/{species}.usd")
    p.SetInstanceable(True)
    xf = UsdGeom.Xformable(p)
    xf.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], 0.0)); xf.AddRotateZOp().Set(float(yaw)); xf.AddScaleOp().Set(Gf.Vec3f(*(3 * [scale * VEG_UNIT])))
    return p


def plant(stage, path, kind, name, xyz, yaw=0.0, scale=1.0):
    p = stage.DefinePrim(path, "Xform")
    p.GetReferences().AddReference(f"{VEG}/{kind}/{name}.usd")
    p.SetInstanceable(True)
    xf = UsdGeom.Xformable(p)
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz)); xf.AddRotateZOp().Set(float(yaw)); xf.AddScaleOp().Set(Gf.Vec3f(*(3 * [scale * VEG_UNIT])))
    return p



PEOPLE = f"{ROOT}/Isaac/People"


def person(stage, path, character, anim, xyz, yaw=0.0, frame=0):
    """Isaac character posed by a real skeletal animation (sampled at `frame`) instead of the T-pose."""
    from pxr import UsdSkel, Usd
    holder = stage.DefinePrim(path, "Xform")          # our transform lives on a wrapper; the asset keeps its own
    xf = UsdGeom.Xformable(holder)
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz)); xf.AddRotateZOp().Set(float(yaw))
    p = stage.DefinePrim(f"{path}/Char", "Xform")
    p.GetReferences().AddReference(f"{PEOPLE}/Characters/{character}/{character}.usd")
    a = stage.DefinePrim(f"{path}_Anim", "SkelAnimation")
    a.GetReferences().AddReference(f"{PEOPLE}/Animations/{anim}.skelanim.usd")
    # bind on every Skeleton inside the character
    for q in Usd.PrimRange(p):
        if q.GetTypeName() == "Skeleton":
            UsdSkel.BindingAPI.Apply(q).CreateAnimationSourceRel().SetTargets([a.GetPath()])
    # freeze the pose at `frame`: copy the sampled joint transforms as default values
    an = UsdSkel.Animation(a)
    for attr in (an.GetTranslationsAttr(), an.GetRotationsAttr(), an.GetScalesAttr()):
        if attr and attr.HasAuthoredValue():
            v = attr.Get(Usd.TimeCode(frame))
            if v is not None:
                attr.Set(v)                       # default (time-independent) value on the local layer
    return p


def prop(stage, path, rel, xyz, yaw=0.0, unit=0.01):
    holder = stage.DefinePrim(path, "Xform")
    xf = UsdGeom.Xformable(holder)
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz)); xf.AddRotateZOp().Set(float(yaw)); xf.AddScaleOp().Set(Gf.Vec3f(unit, unit, unit))
    stage.DefinePrim(f"{path}/Asset", "Xform").GetReferences().AddReference(f"{ROOT}/NVIDIA/Assets/{rel}")
    return holder


def _quat_axis(axis, deg):
    import math
    h = math.radians(deg) / 2
    v = Gf.Vec3d(*axis).GetNormalized() * math.sin(h)
    return Gf.Quatd(math.cos(h), v)


def posed_person(stage, path, character, xyz, yaw=0.0, arm_axis=(1, 0, 0), arm_deg=70.0, elbow_deg=12.0, extra=None):
    """Isaac (Reallusion-rig) character in a relaxed standing pose authored directly on its skeleton:
    rest pose + upper arms rotated down to the sides + a slight elbow bend. `extra`: {joint_suffix: (axis, deg)}."""
    from pxr import UsdSkel, Usd, Vt
    holder = stage.DefinePrim(path, "Xform")
    xf = UsdGeom.Xformable(holder)
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz)); xf.AddRotateZOp().Set(float(yaw))
    p = stage.DefinePrim(f"{path}/Char", "Xform")
    p.GetReferences().AddReference(f"{PEOPLE}/Characters/{character}/{character}.usd")
    for q in Usd.PrimRange(p):
        if q.GetTypeName() != "Skeleton":
            continue
        sk = UsdSkel.Skeleton(q)
        joints = list(sk.GetJointsAttr().Get())
        rest = sk.GetRestTransformsAttr().Get()
        tr, rot, sc = [], [], []
        for j, m in zip(joints, rest):
            mm = Gf.Matrix4d(m)
            t = mm.ExtractTranslation(); r = mm.ExtractRotationQuat()
            name = j.split("/")[-1]
            side = 1 if name.startswith("L_") else -1
            if name in ("L_Upperarm", "R_Upperarm"):
                r = r * Gf.Quatd(_quat_axis(arm_axis, side * arm_deg))
            if name in ("L_Forearm", "R_Forearm"):
                r = r * Gf.Quatd(_quat_axis((0, 0, 1), side * elbow_deg))
            for suffix, (ax, dg) in (extra or {}).items():
                if name == suffix:
                    r = r * Gf.Quatd(_quat_axis(ax, dg))
            tr.append(Gf.Vec3f(t)); rot.append(Gf.Quatf(r)); sc.append(Gf.Vec3h(1, 1, 1))
        a = UsdSkel.Animation.Define(stage, f"{path}_Pose")
        a.CreateJointsAttr(Vt.TokenArray(joints))
        a.CreateTranslationsAttr(Vt.Vec3fArray(tr)); a.CreateRotationsAttr(Vt.QuatfArray(rot)); a.CreateScalesAttr(Vt.Vec3hArray(sc))
        UsdSkel.BindingAPI.Apply(q).CreateAnimationSourceRel().SetTargets([a.GetPath()])
    return holder


CAR_PAINTS = [((0.82, 0.82, 0.80), 0.30, 0.20), ((0.55, 0.56, 0.58), 0.25, 0.75), ((0.02, 0.02, 0.025), 0.22, 0.45),
              ((0.16, 0.17, 0.18), 0.25, 0.70), ((0.06, 0.13, 0.30), 0.25, 0.60), ((0.42, 0.04, 0.04), 0.25, 0.55)]


def _loft(xs, ring, nu=24):
    """Mesh through cross-sections: ring(x) -> (cy, cz, half_w, half_h, e, z_floor, tumble). Quads between rings."""
    import numpy as np
    th = np.linspace(-np.pi, np.pi, nu, endpoint=False); pts = []
    for x in xs:
        cy, cz, a, b, e, zf, tb = ring(x)
        c, sn = np.cos(th), np.sin(th)
        y = a * np.sign(c) * np.abs(c) ** e; z = b * np.sign(sn) * np.abs(sn) ** e
        zz = np.maximum(cz + z, zf); h = np.clip((zz - zf) / max(2 * b, 1e-6), 0, 1)
        y = y * (1 - tb * h)
        pts += [(x, cy + yy, zv) for yy, zv in zip(y, zz)]
    counts, idx = [], []
    for j in range(len(xs) - 1):
        for i in range(nu):
            a0 = j * nu + i; b0 = j * nu + (i + 1) % nu
            counts.append(4); idx += [a0, b0, b0 + nu, a0 + nu]
    return np.array(pts), counts, idx


def sedan_geo(L):
    """(body, greenhouse) meshes of a generic sedan, local +X forward, wheels on the ground at z=0."""
    import numpy as np
    h = L / 2
    def interp(x, kx, kv): return float(np.interp(x, kx, kv))
    def body(x):
        r = max(1 - (abs(x) / h) ** 5, 0.0) ** 0.2                     # rounded nose/tail in plan
        top = interp(x, [-h, -0.36 * L, 0.22 * L, h], [0.98, 1.02, 1.0, 0.80])
        bot = 0.30; rz = max(1 - (abs(x) / h) ** 4, 0.0) ** 0.25
        return 0.0, (top + bot) / 2, 0.9 * r + 1e-3, (top - bot) / 2 * (0.55 + 0.45 * rz) + 1e-3, 0.28, -9, 0.0
    def green(x):
        top = interp(x, [-0.37 * L, -0.19 * L, 0.07 * L, 0.23 * L], [1.0, 1.43, 1.45, 1.0])
        return 0.0, 0.97, 0.80, max(top - 0.97, 1e-3), 0.45, 0.97, 0.22
    xb = np.linspace(-h, h, 28); xg = np.linspace(-0.37 * L, 0.23 * L, 22)
    return _loft(xb, body), _loft(xg, green)


def parked_car_protos(stage, root="/World/CarProtos"):
    """Six generic sedans (one per paint), hidden; parked cars reference them as instances."""
    import geom_util as G
    UsdGeom.Imageable(stage.DefinePrim(root, "Xform")).MakeInvisible()
    glass = omnipbr(stage, f"{root}/Looks/Glass", (0.015, 0.017, 0.02), rough=0.04, metal=0.3)
    tyre = omnipbr(stage, f"{root}/Looks/Tyre", (0.025, 0.025, 0.025), rough=0.8)
    rim = omnipbr(stage, f"{root}/Looks/Rim", (0.6, 0.6, 0.62), rough=0.3, metal=1.0)
    head = omnipbr(stage, f"{root}/Looks/Head", (0.9, 0.9, 0.88), rough=0.1)
    tail = omnipbr(stage, f"{root}/Looks/Tail", (0.35, 0.02, 0.02), rough=0.15)
    protos = []
    for c, (col, rough, metal) in enumerate(CAR_PAINTS):
        p = f"{root}/C{c}"; stage.DefinePrim(p, "Xform")
        paint = omnipbr(stage, f"{root}/Looks/Paint{c}", col, rough=rough, metal=metal)
        L = 4.6 + 0.25 * (c % 3)          # length varies a little between models (local +X is forward)
        bgeo, ggeo = sedan_geo(L)
        G.add_mesh(stage, f"{p}/Body", bgeo, paint)
        G.add_mesh(stage, f"{p}/Glass", ggeo, glass)
        G.add_mesh(stage, f"{p}/Roof", G.superellipsoid(-0.06 * L, 0, 1.425, 0.12 * L, 0.6, 0.035, e_v=0.5, e_h=0.3, nu=24, nv=8), paint)
        for sx in (-1, 1):
            for sy in (-1, 1):
                G.cyl(stage, f"{p}/Tyre_{sx+1}{sy+1}", (sx * L * 0.31, sy * 0.80, 0.33), 0.33, 0.22, "Y", tyre)
                G.cyl(stage, f"{p}/Rim_{sx+1}{sy+1}", (sx * L * 0.31, sy * 0.915, 0.33), 0.21, 0.01, "Y", rim)
            G.box(stage, f"{p}/Head_{sx+1}", (L / 2 - 0.08, sx * 0.62, 0.78), (0.1, 0.32, 0.09), head)
            G.box(stage, f"{p}/Tail_{sx+1}", (-L / 2 + 0.06, sx * 0.62, 0.82), (0.1, 0.34, 0.08), tail)
        protos.append(p)
    return protos


def parked_car(stage, path, proto, xy, yaw):
    pr = stage.DefinePrim(path, "Xform")
    pr.GetReferences().AddInternalReference(proto); pr.SetInstanceable(True)
    UsdGeom.Imageable(pr).MakeVisible()
    xf = UsdGeom.Xformable(pr); xf.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], 0.0)); xf.AddRotateZOp().Set(float(yaw))
    return pr
