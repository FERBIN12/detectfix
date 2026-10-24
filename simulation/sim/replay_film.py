"""Film replay: re-fly the recorded drone + ROV mission (rosbag poses, video/replay_*.npy) through the Isaac world at a
locked 30 fps with cinematic cameras, so footage is smooth whatever the render cost. Called from charles_dam_world.py:

    REPLAY=1 SHOTS=a,b HEADLESS=1 OMNI_KIT_ACCEPT_EULA=YES ~/IsaacLab/env_isaaclab/bin/python charles_dam_world.py

Writes video/replay/<shot>/{main,drgb,rcam,sonar}/NNNNN.jpg. Shots are defined in SHOTS below in mission time.
"""
import os, math, time
import numpy as np


def run(g):
    st, rep, Gf, Usd, UsdGeom, RT, cw = g["st"], g["rep"], g["Gf"], g["Usd"], g["UsdGeom"], g["RT"], g["cw"]
    import cv2
    H = os.path.dirname(os.path.abspath(__file__)); V = os.path.join(H, "..", "video")
    D = np.load(os.path.join(V, "replay_drone.npy")); R = np.load(os.path.join(V, "replay_rov.npy"))   # t, x y z, qx qy qz qw

    def smooth(A, k=3):                                         # Gaussian smoothing of the 7 Hz samples (positions only)
        w = np.exp(-0.5 * (np.arange(-3 * k, 3 * k + 1) / k) ** 2); w /= w.sum(); B = A.copy()
        for j in (1, 2, 3):
            B[:, j] = np.convolve(np.pad(A[:, j], 3 * k, mode="edge"), w, mode="valid")
        return B
    D, R = smooth(D, 2), smooth(R, 2)

    def at(A, t):
        i = int(np.clip(np.searchsorted(A[:, 0], t) - 1, 0, len(A) - 2)); a = (t - A[i, 0]) / max(A[i + 1, 0] - A[i, 0], 1e-6); a = min(max(a, 0.0), 1.0)
        p = A[i, 1:4] * (1 - a) + A[i + 1, 1:4] * a
        q0, q1 = A[i, 4:8], A[i + 1, 4:8]
        if np.dot(q0, q1) < 0: q1 = -q1
        q = q0 * (1 - a) + q1 * a; q /= np.linalg.norm(q)
        return p, q

    def mat(p, q):
        r = Gf.Rotation(Gf.Quatd(float(q[3]), Gf.Vec3d(float(q[0]), float(q[1]), float(q[2]))))
        return Gf.Matrix4d().SetRotate(r) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*map(float, p)))

    def yaw_of(q):
        x, y, z, w = q; return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # cinematic camera + extra sensor render products
    cine = UsdGeom.Camera.Define(st, "/World/Cine"); cine.CreateClippingRangeAttr(Gf.Vec2f(0.03, 3000)); cine_f = cine.CreateFocalLengthAttr(20.0)
    cine.CreateHorizontalApertureAttr(36.0); cine.CreateVerticalApertureAttr(20.25); c_op = UsdGeom.Xformable(cine).AddTransformOp()
    rp_m = rep.create.render_product("/World/Cine", (1920, 1080)); a_m = rep.AnnotatorRegistry.get_annotator("LdrColor"); a_m.attach([rp_m])
    rp_rc = rep.create.render_product(g["ROV_CAM"], (640, 360))
    a_rc = rep.AnnotatorRegistry.get_annotator("LdrColor"); a_rc.attach([rp_rc]); a_rd = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); a_rd.attach([rp_rc])
    rp_s = rep.create.render_product(g["ROV_SON"], (g["Sonar"].W, g["Sonar"].H))
    a_sd = rep.AnnotatorRegistry.get_annotator("distance_to_camera"); a_sd.attach([rp_s]); a_sn = rep.AnnotatorRegistry.get_annotator("normals"); a_sn.attach([rp_s])
    SON = g["Sonar"](np.random.default_rng(3)); murk = g["murk"]

    def look(eye, tgt, up=(0, 0, 1)):
        m = Gf.Matrix4d(); m.SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, tgt)), Gf.Vec3d(*up)); c_op.Set(m.GetInverse())

    def fwd(q):
        y = yaw_of(q); return np.array([math.cos(y), math.sin(y), 0.0])

    # ---- shots: name -> (t0, t1, speed, focal_mm, camera(t, drone_p, drone_q, rov_p, rov_q) -> (eye, target), sensors)
    E = RT.CANAL_W / 2
    def drone_chase(t, dp, dq, rp, rq):
        f = fwd(dq); s = np.array([-f[1], f[0], 0]); return dp - 5.5 * f + 1.6 * s + np.array([0, 0, 1.4]), dp + np.array([0, 0, -0.2])
    def drone_crane(t, dp, dq, rp, rq):
        a = (t - 0.0) / 14.0; eye = cw(-80 + 14 * a, -9 + 4 * a, 14 - 8 * a); return eye, dp
    def over_under(t, dp, dq, rp, rq):
        return cw(-46.5, 1.8, 0.04), cw(-38.0, E - 0.4, -0.3)
    def under_deck(t, dp, dq, rp, rq):
        return cw(-4.0, -3.5, 1.0), dp + np.array([0, 0, 0.4])
    def rov_chase(t, dp, dq, rp, rq):
        f = fwd(rq); s = np.array([-f[1], f[0], 0]); return rp - 2.3 * f + 1.0 * s + np.array([0, 0, 0.6]), rp + 0.7 * f
    def rov_side(t, dp, dq, rp, rq):
        f = np.array([*RT.C, 0.0]); n = np.array([*(-RT.R), 0.0]); return rp + 2.6 * n - 1.2 * f + np.array([0, 0, 0.5]), rp + 0.8 * f + np.array([0, 0, -0.2])
    def landing(t, dp, dq, rp, rq):
        return RT.LAUNCH + cw(-5.0, 4.0, 2.8), dp + np.array([0, 0, -0.3])
    SHOTS = {
        "crane":     (0.5, 12.0, 1.0, 22.0, drone_crane),
        "chase":     (4.0, 26.0, 1.0, 20.0, drone_chase),
        "underdeck": (40.0, 56.0, 1.0, 16.0, under_deck),
        "rovdive":   (28.0, 44.0, 1.0, 16.0, rov_chase),
        "rovblock":  (60.0, 78.0, 1.0, 16.0, rov_side),
        "rovscour":  (228.0, 246.0, 1.0, 16.0, rov_chase),
        "landing":   (120.0, 136.0, 1.0, 24.0, landing),
    }
    HERO = {   # frozen pose time, camera offset in the vehicle frame (fwd, left, up), look offset, sensors (vehicle-frame points)
        "dronehero": (8.0, "drone", (1.35, -0.95, 0.30), (0.95, -0.65, 0.18), (0.0, 0.0, -0.03), {
            "lidar": (0.0, 0.0, 0.17), "gnss": (-0.09, 0.0, 0.155), "gimbal": (0.03, 0.0, -0.17), "flood": (0.16, -0.11, -0.005), "nav": (0.188, -0.188, 0.025)}),
        "rovhero": (64.0, "rov", (0.75, 0.85, 0.22), (0.55, 0.70, 0.16), (0.0, 0.0, -0.02), {
            "sonar": (0.137, 0.0, 0.165), "camera": (0.21, 0.0, 0.05), "lumen": (0.235, 0.12, 0.10), "dvl": (-0.04, 0.0, -0.14)}),
    }
    want = os.environ.get("SHOTS", ",".join(list(HERO) + list(SHOTS))).split(",")
    gyaw_up = None
    import json
    for name in [w for w in want if w in HERO]:
        tf, veh, e0, e1, lk, pts = HERO[name]
        out = os.path.join(V, "replay", name); os.makedirs(os.path.join(out, "main"), exist_ok=True)
        dp, dq = at(D, tf); rp, rq = at(R, tf)
        if veh == "drone": dp = dp.copy(); dp[2] = max(dp[2], 1.6)
        g["d_op"].Set(mat(dp, dq)); g["ROV_OP"].Set(mat(rp, rq))
        vp, vq = (dp, dq) if veh == "drone" else (rp, rq)
        Mv = np.array(mat(vp, vq)); to_w = lambda v: np.asarray(v + (1.0,)) @ Mv
        cine_f.Set(24.0); n = 300; ema = None; proj = {}
        for k in range(-20, n):
            t = tf + max(k, 0) / 30.0
            for pop, cc, sgn in g["PROPS"]:
                pop.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), (sgn * 2900 * t) % 360)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*cc)))
            g["set_water_normal"](int(t * 15))
            a_ = max(k, 0) / n; e = tuple(e0[i] + (e1[i] - e0[i]) * a_ for i in range(3))
            eye = vp + cw(*e); tg = vp + np.asarray(lk); look(eye, tg)          # camera offset in the canal frame (c, r, z)
            rep.orchestrator.step(rt_subframes=2)
            if k < 0: continue
            im = np.asarray(a_m.get_data())
            if im.size: cv2.imwrite(os.path.join(out, "main", f"{k:05d}.jpg"), im[:, :, 2::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
            if k % 30 == 0 or k == n - 1:                                # sensor points -> pixels (USD camera looks down -Z)
                Mc = np.array(UsdGeom.Xformable(cine).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse())
                fx = 1920 * 24.0 / 36.0
                fr = {}
                for nm, v in pts.items():
                    pc = np.asarray(tuple(to_w(v)[:3]) + (1.0,)) @ Mc
                    fr[nm] = [960 + fx * pc[0] / -pc[2], 540 - fx * pc[1] / -pc[2]]
                proj[k] = fr
        json.dump(proj, open(os.path.join(out, "callouts.json"), "w"))
        print(f"SHOT {name} {n} frames", flush=True)
    for name in [w for w in want if w in SHOTS]:
        t0, t1, sp, foc, camf = SHOTS[name]
        out = os.path.join(V, "replay", name)
        for d_ in ("main", "drgb", "rcam", "sonar"): os.makedirs(os.path.join(out, d_), exist_ok=True)
        cine_f.Set(float(foc)); n = int((t1 - t0) / sp * 30); ema = None; tw = time.time()
        for k in range(-20, n):                                   # 20 warm-up frames so the first saved frame is converged
            t = t0 + max(k, 0) * sp / 30.0
            dp, dq = at(D, t); rp, rq = at(R, t)
            g["d_op"].Set(mat(dp, dq)); g["ROV_OP"].Set(mat(rp, rq))
            c_, r_ = float(np.dot(dp[:2], RT.C)), float(np.dot(dp[:2], RT.R))
            if RT.under_deck(dp): tgt = dp + np.array([*(RT.C * 0.5), 0.0]) + np.array([0, 0, 2.0])
            else: tgt = cw(c_, math.copysign(E, r_ if abs(r_) > 0.5 else 1.0), max(dp[2] - 0.3, 0.6))
            yw = yaw_of(dq); gyaw = math.atan2(tgt[1] - dp[1], tgt[0] - dp[0]) - yw
            pitch = max(-25.0, -math.degrees(math.atan2(tgt[2] - (dp[2] - 0.17), max(np.hypot(*(tgt - dp)[:2]), 0.1))))
            g["gim_op"].Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch)) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(gyaw))))
            for pop, cc, sgn in g["PROPS"]:
                pop.Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), (sgn * 2900 * t) % 360)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*cc)))
            for car in g["CARS"]:
                u = ((car[3] + car[2] * car[4] * t + 80) % 160) - 80; car[0].Set(g["car_pose"](u, car[1]))
            g["set_water_normal"](int(t * 15))
            eye, tg = camf(t, dp, dq, rp, rq); eye = np.asarray(eye, float); tg = np.asarray(tg, float)
            ema = (eye, tg) if ema is None else (0.85 * ema[0] + 0.15 * eye, 0.85 * ema[1] + 0.15 * tg)   # camera inertia
            look(*ema)
            rep.orchestrator.step(rt_subframes=2)
            if k < 0: continue
            im = np.asarray(a_m.get_data())
            if im.size: cv2.imwrite(os.path.join(out, "main", f"{k:05d}.jpg"), im[:, :, 2::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
            if k % 2 == 0:
                rgb = np.asarray(g["a_rgb"].get_data())
                if rgb.size: cv2.imwrite(os.path.join(out, "drgb", f"{k:05d}.jpg"), rgb[:, :, 2::-1], [cv2.IMWRITE_JPEG_QUALITY, 90])
                rc, rd = np.asarray(a_rc.get_data()), np.asarray(a_rd.get_data(), np.float32)
                if rc.size and rd.size:
                    mm = murk(rc[:, :, :3], rd, -rp[2]) if rp[2] < -0.05 else rc[:, :, :3]
                    cv2.imwrite(os.path.join(out, "rcam", f"{k:05d}.jpg"), np.ascontiguousarray(mm)[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 90])
                Ms = UsdGeom.Xformable(st.GetPrimAtPath(g["ROV_SON"])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                polar, _ = SON.ping(a_sd.get_data(), a_sn.get_data(), np.array(Ms.ExtractRotationMatrix()).T, np.array(Ms.ExtractTranslation()))
                if polar is not None: cv2.imwrite(os.path.join(out, "sonar", f"{k:05d}.jpg"), SON.fan(polar)[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f"SHOT {name} {n} frames in {time.time() - tw:.0f}s", flush=True)
