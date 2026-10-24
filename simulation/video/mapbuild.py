"""Map-from-scratch animation from the recorded bag: every lidar (drone) and sonar (ROV) point appears at the time it was
measured, mission sped up, slow orbit, both paths growing, the drone + ROV camera frames from that moment alongside.
    source /opt/ros/jazzy/setup.bash && ~/IsaacLab/env_isaaclab/bin/python mapbuild.py   -> build3/mapbuild.mp4
"""
import os, sys, bisect, subprocess, math
import numpy as np, cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs_py import point_cloud2 as pc2
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(H, "..", "sim")); import route_dam as RT
BAG = os.path.expanduser("~/hackachusetts/bags/run_20261024_134808")
W, HH, FPS, SECS = 1920, 1080, 30, 11.0
FD = os.path.expanduser("~/biofouling-yc/video/assets/")


def tmat(tr):
    q = tr.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]; return T


r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/lidar/points", "/rov/sonar/points", "/odom", "/rov/truth/odom", "/camera/rgb", "/rov/camera"]))
tf = {"base_link": ([], []), "os0_lidar": ([], [])}; lid, son, dpath, rpath, dimg, rimg = [], [], [], [], [], []
T0 = None
while r.has_next():
    topic, data, t = r.read_next(); t *= 1e-9; T0 = t if T0 is None else T0; t -= T0
    if topic == "/tf":
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.child_frame_id in tf: tf[tr.child_frame_id][0].append(t); tf[tr.child_frame_id][1].append(tmat(tr.transform))
    elif topic == "/lidar/points":
        lid.append((t, pc2.read_points_numpy(deserialize_message(data, PointCloud2), field_names=("x", "y", "z"))[::10]))
    elif topic == "/rov/sonar/points":
        son.append((t, pc2.read_points_numpy(deserialize_message(data, PointCloud2), field_names=("x", "y", "z"))[::3]))
    elif topic in ("/odom", "/rov/truth/odom"):
        p = deserialize_message(data, Odometry).pose.pose.position; (dpath if topic == "/odom" else rpath).append((t, p.x, p.y, p.z))
    else:
        m = deserialize_message(data, Image); im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        (dimg if topic == "/camera/rgb" else rimg).append((t, cv2.resize(im, (480, 270))[:, :, ::-1].copy()))
print("decoded", len(lid), "lidar,", len(son), "sonar,", len(dimg), len(rimg), "images", flush=True)


def near(child, t):
    ts, Ts = tf[child]; i = min(max(bisect.bisect_left(ts, t), 0), len(ts) - 1); return Ts[i]


Pt, Pk, Pp = [], [], []
for t, P in lid:
    if not len(P): continue
    T = near("base_link", t) @ near("os0_lidar", t); Q = P @ T[:3, :3].T + T[:3, 3]; Pp.append(Q); Pt.append(np.full(len(Q), t)); Pk.append(np.zeros(len(Q), np.int8))
for t, P in son:
    if len(P): Pp.append(P); Pt.append(np.full(len(P), t)); Pk.append(np.ones(len(P), np.int8))
P = np.concatenate(Pp).astype(np.float32); Tm = np.concatenate(Pt); K = np.concatenate(Pk)
c = P[:, :2] @ RT.C; rr = P[:, :2] @ RT.R
ok = (c > -75) & (c < 40) & (np.abs(rr) < 22) & (P[:, 2] > -7) & (P[:, 2] < 14) & ((K == 1) | (P[:, 2] > 0.12))
P, Tm, K = P[ok], Tm[ok], K[ok]
key = np.floor(P / 0.12).astype(np.int64); key = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
order = np.argsort(Tm, kind="stable"); _, first = np.unique(key[order], return_index=True); sel = order[first]
P, Tm, K = P[sel], Tm[sel], K[sel]; o = np.argsort(Tm); P, Tm, K = P[o], Tm[o], K[o]
print("points", len(P), "lidar", int((K == 0).sum()), "sonar", int((K == 1).sum()), flush=True)
z = P[:, 2]
hc = cv2.applyColorMap(np.clip((z - 0.0) / 9.0 * 255, 0, 255).astype(np.uint8)[:, None], cv2.COLORMAP_TURBO)[:, 0, ::-1]
sc = cv2.applyColorMap(np.clip((-z) / 5.5 * 255, 0, 255).astype(np.uint8)[:, None], cv2.COLORMAP_AUTUMN)[:, 0, ::-1]
COL = np.where((K == 1)[:, None], sc, hc).astype(np.uint8)
D = np.array(dpath); R_ = np.array(rpath); TEND = max(Tm.max(), D[-1, 0])
CEN = RT.cw(-18.0, 0.0, -0.5)


def cam(f):
    u = f / (SECS * FPS); a = math.radians(-135 + 30 * u); dist = 92 - 10 * u
    eye = CEN + np.array([dist * math.cos(a), dist * math.sin(a), 46 - 6 * u])
    fw = CEN - eye; fw /= np.linalg.norm(fw); rt = np.cross(fw, [0, 0, 1.0]); rt /= np.linalg.norm(rt); up = np.cross(rt, fw)
    return eye, fw, rt, up


def proj(X, eye, fw, rt, up, foc=1500.0):
    d = X - eye; zc = d @ fw; x = d @ rt; y = d @ up
    return W * 0.40 + foc * x / zc, HH * 0.52 - foc * y / zc, zc


from PIL import Image as PImage, ImageDraw, ImageFont
fB = ImageFont.truetype(FD + "BarlowCondensed-SemiBold.ttf", 40); fS = ImageFont.truetype(FD + "Barlow-Regular.ttf", 28); fM = ImageFont.truetype(FD + "IBMPlexMono-Medium.ttf", 30)
out = os.path.join(H, "build3", "mapbuild.mp4"); os.makedirs(os.path.dirname(out), exist_ok=True)
pp = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{HH}", "-r", str(FPS), "-i", "-",
                       "-c:v", "libx264", "-preset", "veryfast", "-crf", "17", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
NF = int(SECS * FPS)
water = [RT.cw(c_, r_, 0.0) for c_, r_ in ((-75, -6.5), (40, -6.5), (40, 6.5), (-75, 6.5))]
for f in range(NF):
    u = min(1.0, (f / (NF - 1)) / 0.94); TD = 134.0
    tm = TD * (u / 0.55) if u < 0.55 else TD + (TEND - TD) * ((u - 0.55) / 0.45)   # drone phase gets half the time
    n = np.searchsorted(Tm, tm); eye, fw, rt, up = cam(f)
    img = np.zeros((HH, W, 3), np.uint8); img[:] = (8, 12, 18)
    x, y, zc = proj(P[:n], eye, fw, rt, up); v = (zc > 1) & (x >= 0) & (x < W - 1) & (y >= 0) & (y < HH - 1)
    o = np.argsort(-zc[v]); xi = x[v][o].astype(int); yi = y[v][o].astype(int); cc = COL[:n][v][o]
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)): img[yi + dy, xi + dx] = cc
    wp = np.array([proj(np.array(q), eye, fw, rt, up)[:2] for q in water], np.int32)
    ov = img.copy(); cv2.fillPoly(ov, [wp], (40, 90, 120)); img = cv2.addWeighted(ov, 0.18, img, 0.82, 0)
    for path, col in ((D, (110, 205, 255)), (R_, (255, 150, 40))):
        q = path[path[:, 0] <= tm][:, 1:4]
        if len(q) > 1:
            px, py, pz = proj(q, eye, fw, rt, up); pts = np.stack([px, py], 1).astype(np.int32)
            cv2.polylines(img, [pts], False, col, 4, cv2.LINE_AA); cv2.circle(img, tuple(pts[-1]), 11, col, -1, cv2.LINE_AA); cv2.circle(img, tuple(pts[-1]), 16, (255, 255, 255), 2, cv2.LINE_AA)
    pim = PImage.fromarray(img); d = ImageDraw.Draw(pim)
    for lst, lab, yy in ((dimg, "DRONE CAMERA", 120), (rimg, "ROV CAMERA", 460)):
        ts = [a for a, _ in lst]; i = min(max(bisect.bisect_left(ts, min(tm, 104.0) if "DRONE" in lab else tm), 0), len(lst) - 1)
        d.rectangle([W - 560 - 3, yy - 3, W - 80 + 3, yy + 270 + 3], fill=(110, 205, 255) if "DRONE" in lab else (255, 150, 40))
        pim.paste(PImage.fromarray(lst[i][1]), (W - 560, yy)); d.text((W - 560, yy - 12), lab, font=fB, fill=(240, 244, 248), anchor="ls")
    nl, ns = int((K[:n] == 0).sum()), int((K[:n] == 1).sum())
    d.text((100, 110), "BUILDING THE MAP · FROM SCRATCH", font=fB, fill=(110, 205, 255), anchor="ls")
    d.text((100, 160), f"mission time T+{int(tm // 60):02d}:{int(tm % 60):02d}  (sped up)", font=fM, fill=(240, 244, 248), anchor="ls")
    d.rounded_rectangle([80, 900, 560, 1030], 12, fill=(6, 10, 16))
    d.text((100, 960), f"lidar above  {nl:>9,}", font=fM, fill=(110, 205, 255), anchor="ls")
    d.text((100, 1004), f"sonar below  {ns:>9,}", font=fM, fill=(255, 150, 40), anchor="ls")
    pp.stdin.write(np.asarray(pim).tobytes())
    if f % 60 == 0: print("frame", f, n, flush=True)
pp.stdin.close(); pp.wait(); print("MAPBUILD", out)
