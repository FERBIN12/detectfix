"""Holybro X500 V2 inspection drone in Isaac, built from PX4's own meshes (PX4-gazebo-models, BSD-3-Clause):
NXP HoverGames carbon frame + FMUK66, 5010 motors, 13x4.5 props, Yuneec CGO3 gimbal camera; plus the inspection payload
(Ouster OS0 lidar on a top mast, RTK GNSS puck, LED work lights).

    from x500 import build_x500
    props, gimbal_op, cam_path = build_x500(stage, "/World/Drone", PR)

props: [(xform_op, rotor_xyz, spin_sign)] to animate; gimbal_op: transform op of the pitch/yaw pivot; cam_path: USD camera.
"""
import os
import numpy as np
import trimesh
from pxr import UsdGeom, UsdShade, Sdf, Gf, Vt, UsdLux

A = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "x500")


def _mat(st, path, PR, color, rough, metal=0.0, tex=None):
    if tex:
        m = PR.mdl_material(st, path, "OmniPBR.mdl", "OmniPBR", {"reflection_roughness_constant": float(rough), "metallic_constant": float(metal)})
        sh = UsdShade.Shader.Get(st, path + "/Shader")
        x = sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset); x.Set(Sdf.AssetPath(tex)); x.GetAttr().SetColorSpace("sRGB")
        return m
    return PR.omnipbr(st, path, color, rough, metal)


def _add(st, path, g, mat, scale=1.0, xyz=(0, 0, 0), rpy_z=0.0):
    """trimesh geometry -> UsdGeom.Mesh with normals + UVs."""
    me = UsdGeom.Mesh.Define(st, path)
    v = np.asarray(g.vertices, np.float64) * scale
    me.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(g.faces)))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(np.asarray(g.faces, np.int32).ravel().tolist()))
    me.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(g.vertex_normals, np.float32))); me.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    me.CreateSubdivisionSchemeAttr("none")
    uv = getattr(g.visual, "uv", None)
    if uv is not None and len(uv) == len(v):
        UsdGeom.PrimvarsAPI(me.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
            Vt.Vec2fArray.FromNumpy(np.asarray(uv, np.float32)))
    xf = UsdGeom.Xformable(me)
    if any(xyz): xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, xyz)))
    if rpy_z: xf.AddRotateZOp().Set(float(np.degrees(rpy_z)))
    UsdShade.MaterialBindingAPI.Apply(me.GetPrim()).Bind(mat)
    return me


def _baked(scene):
    """geometries with their scene-graph transforms applied (the .dae files carry unit scale + axis in the node graph)."""
    out = []
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph[node]
        g = scene.geometry[gname].copy(); g.apply_transform(T); out.append((gname, g))
    return out


def build_x500(st, root, PR):
    UsdGeom.Xform.Define(st, root)
    M = dict(cf=_mat(st, root + "/Looks/CarbonFiber", PR, (0.05, 0.05, 0.05), 0.25, tex=os.path.join(A, "x500_base/meshes/CF.png")),
             foam=_mat(st, root + "/Looks/Foam", PR, (0.55, 0.55, 0.55), 0.85), rubber=_mat(st, root + "/Looks/Rubber", PR, (0.01, 0.01, 0.01), 0.8),
             metal=_mat(st, root + "/Looks/Metal", PR, (0.32, 0.32, 0.33), 0.3, 1.0), rails=_mat(st, root + "/Looks/Rails", PR, (0.1, 0.1, 0.1), 0.6),
             pcb=_mat(st, root + "/Looks/FMU", PR, (0.85, 0.85, 0.85), 0.4), motor=_mat(st, root + "/Looks/Motor", PR, (0.2, 0.2, 0.21), 0.35, 1.0),
             bell=_mat(st, root + "/Looks/Bell", PR, (0.03, 0.03, 0.03), 0.3, 0.8), prop=_mat(st, root + "/Looks/Prop", PR, (0.02, 0.02, 0.02), 0.35),
             gim=_mat(st, root + "/Looks/Gimbal", PR, (0.75, 0.76, 0.78), 0.3, 0.2), lens=_mat(st, root + "/Looks/Lens", PR, (0.01, 0.01, 0.015), 0.05, 0.6),
             lidar=_mat(st, root + "/Looks/Lidar", PR, (0.18, 0.18, 0.19), 0.3, 0.8), gnss=_mat(st, root + "/Looks/GNSS", PR, (0.9, 0.9, 0.9), 0.4))
    by_name = {"LandingFoam": "foam", "LandingRubber": "rubber", "CarbonFiber": "cf", "Metal": "metal", "RailsRubber": "rails", "FMUK66": "pcb"}
    frame = trimesh.load(os.path.join(A, "x500_base/meshes/NXP-HGD-CF.dae"), force="scene")
    fr = UsdGeom.Xform.Define(st, root + "/Frame"); fx = UsdGeom.Xformable(fr)
    fx.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.025)); fx.AddRotateZOp().Set(180.0)
    for i, (n, g) in enumerate(_baked(frame)):
        key = next((v for k, v in by_name.items() if n.startswith(k)), "cf")
        _add(st, f"{root}/Frame/P{i}", g, M[key])
    mb = trimesh.load(os.path.join(A, "x500_base/meshes/5010Base.dae"), force="scene")
    bell = trimesh.load(os.path.join(A, "x500_base/meshes/5010Bell.dae"), force="scene")
    pcw = trimesh.load(os.path.join(A, "x500_base/meshes/1345_prop_cw.stl")); pccw = trimesh.load(os.path.join(A, "x500_base/meshes/1345_prop_ccw.stl"))
    props = []
    for i, (x, y, ccw) in enumerate(((0.174, -0.174, True), (-0.174, 0.174, True), (0.174, 0.174, False), (-0.174, -0.174, False))):
        for j, (_, g) in enumerate(_baked(mb)):
            _add(st, f"{root}/MotorBase{i}_{j}", g, M["motor"], xyz=(x, y, 0.032), rpy_z=-0.45)
        r = UsdGeom.Xform.Define(st, f"{root}/Rotor{i}"); op = UsdGeom.Xformable(r).AddTransformOp()
        for j, (_, g) in enumerate(_baked(bell)):
            _add(st, f"{root}/Rotor{i}/Bell{j}", g, M["bell"] if j == 0 else M["motor"], xyz=(0, 0, 0.06 - 0.032))
        _add(st, f"{root}/Rotor{i}/Prop", pccw if ccw else pcw, M["prop"], scale=0.8461538, xyz=(-0.022, -0.14638, 0.06 - 0.016))
        props.append((op, (x, y, 0.0), 1 if ccw else -1))
    # CGO3 gimbal under the nose; meshes are in mm and already in the gimbal model frame
    gp = UsdGeom.Xform.Define(st, root + "/GimbalMount"); UsdGeom.Xformable(gp).AddTranslateOp().Set(Gf.Vec3d(0.06, 0, -0.01))
    _add(st, root + "/GimbalMount/Mount", trimesh.load(os.path.join(A, "gimbal/meshes/cgo3_mount_remeshed_v1.stl")), M["gim"], scale=0.001)
    piv = UsdGeom.Xform.Define(st, root + "/GimbalMount/Pivot"); UsdGeom.Xformable(piv).AddTranslateOp().Set(Gf.Vec3d(-0.035, 0, -0.162))
    gimbal_op = UsdGeom.Xformable(UsdGeom.Xform.Define(st, root + "/GimbalMount/Pivot/Rot")).AddTransformOp()
    off = (0.035, 0, 0.162)                                                     # mesh coords are model-frame: undo the pivot offset
    for nm, f in (("VArm", "cgo3_vertical_arm_remeshed_v1.stl"), ("HArm", "cgo3_horizontal_arm_remeshed_v1.stl"), ("Cam", "cgo3_camera_remeshed_v1.stl")):
        _add(st, f"{root}/GimbalMount/Pivot/Rot/{nm}", trimesh.load(os.path.join(A, "gimbal/meshes", f)), M["gim"], scale=0.001, xyz=off)
    cam = UsdGeom.Camera.Define(st, root + "/GimbalMount/Pivot/Rot/Camera")
    cam.CreateFocalLengthAttr(8.0); cam.CreateHorizontalApertureAttr(12.8); cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 500))
    UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0.048, 0, 0, 1))   # looks along pivot +x, from the lens (front of the housing)
    # payload: lidar mast + RTK GNSS + LED work lights
    def cyl(path, xyz, r, h, m):
        c = UsdGeom.Cylinder.Define(st, path); c.CreateRadiusAttr(r); c.CreateHeightAttr(h)
        UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*xyz)); UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(m)
    cyl(root + "/LidarMast", (0, 0, 0.09), 0.012, 0.07, M["cf"]); cyl(root + "/LidarBody", (0, 0, 0.16), 0.043, 0.075, M["lidar"])
    cyl(root + "/LidarCap", (0, 0, 0.2), 0.044, 0.006, M["bell"])
    cyl(root + "/GNSSMast", (-0.09, 0, 0.1), 0.006, 0.09, M["cf"]); cyl(root + "/GNSS", (-0.09, 0, 0.15), 0.04, 0.018, M["gnss"])
    # --- lights: two forward LED floodlights (Lume Cube-class housings on the front arms), one upward soffit light,
    #     navigation lights (red port / green starboard at the arm tips) and a white anti-collision strobe at the back
    led = PR.omnipbr(st, root + "/Looks/LEDface", (1.0, 1.0, 1.0), 0.2, emissive=((1.0, 0.97, 0.9), 30000.0))
    red = PR.omnipbr(st, root + "/Looks/NavRed", (1.0, 0.05, 0.03), 0.3, emissive=((1.0, 0.04, 0.02), 20000.0))
    grn = PR.omnipbr(st, root + "/Looks/NavGreen", (0.05, 1.0, 0.1), 0.3, emissive=((0.04, 1.0, 0.08), 20000.0))
    wht = PR.omnipbr(st, root + "/Looks/Strobe", (1.0, 1.0, 1.0), 0.3, emissive=((1.0, 1.0, 1.0), 25000.0))
    housing = M["bell"]

    def bx(path, xyz, size, m):
        c = UsdGeom.Cube.Define(st, path); c.CreateSizeAttr(1.0); x = UsdGeom.Xformable(c)
        x.AddTranslateOp().Set(Gf.Vec3d(*xyz)); x.AddScaleOp().Set(Gf.Vec3f(*size)); UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(m)

    def spot(path, xyz, yaw_pitch, intensity, cone):
        L = UsdLux.SphereLight.Define(st, path); L.CreateRadiusAttr(0.015); L.CreateIntensityAttr(float(intensity))
        L.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92)); UsdLux.ShapingAPI.Apply(L.GetPrim()).CreateShapingConeAngleAttr(float(cone))
        UsdLux.ShapingAPI(L.GetPrim()).CreateShapingConeSoftnessAttr(0.3)
        lx = UsdGeom.Xformable(L); lx.AddTranslateOp().Set(Gf.Vec3d(*xyz)); lx.AddRotateZOp().Set(float(yaw_pitch[0])); lx.AddRotateYOp().Set(float(yaw_pitch[1]))

    for i, y in enumerate((-0.11, 0.11)):                                   # forward floodlights (lights point along local -Z: Y=-90 -> +x)
        bx(f"{root}/Flood{i}/Housing", (0.13, y, -0.005), (0.045, 0.045, 0.045), housing)
        bx(f"{root}/Flood{i}/Lens", (0.153, y, -0.005), (0.002, 0.036, 0.036), led)
        spot(f"{root}/Flood{i}/Beam", (0.16, y, -0.005), (0.0, -90.0), 1.6e6, 32.0)
    bx(f"{root}/Up/Housing", (0.05, 0.0, 0.055), (0.04, 0.04, 0.02), housing)  # upward soffit light (deck undersides)
    bx(f"{root}/Up/Lens", (0.05, 0.0, 0.066), (0.032, 0.032, 0.002), led)
    spot(f"{root}/Up/Beam", (0.05, 0.0, 0.07), (0.0, 180.0), 1.0e6, 50.0)
    for i, (x, y, m) in enumerate(((0.174, 0.174, red), (-0.174, 0.174, red), (0.174, -0.174, grn), (-0.174, -0.174, grn))):
        bx(f"{root}/Nav{i}", (x * 1.08, y * 1.08, 0.025), (0.014, 0.014, 0.01), m)   # port (+y) red, starboard (-y) green
    bx(f"{root}/Strobe", (-0.11, 0.0, 0.04), (0.016, 0.016, 0.012), wht)
    sL = UsdLux.SphereLight.Define(st, root + "/StrobeLight"); sL.CreateRadiusAttr(0.01); sL.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(sL).AddTranslateOp().Set(Gf.Vec3d(-0.11, 0.0, 0.05))
    return props, gimbal_op, root + "/GimbalMount/Pivot/Rot/Camera"   # strobe light prim: root + '/StrobeLight' (blink it in the loop)
