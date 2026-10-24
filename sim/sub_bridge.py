"""MAVLink bridge for the BlueROV2 Heavy: ArduSub SITL <-> simulated DVL <-> cooperative tasking <-> Isaac Sim.

    ~/IsaacLab/env_isaaclab/bin/python sub_bridge.py      (ArduSub SITL instance 1 on tcp:5770; QGC can use tcp:5772)

1. No GPS underwater (sitl/sub_dvl.parm: GPS_TYPE 0). A Water Linked A50-class DVL is simulated from the SITL vehicle's
   true motion (SIMSTATE) with scale error, white noise and bottom lock (needs the bed within 0.05-50 m); it is sent to
   the autopilot as VISION_POSITION_DELTA, exactly what the BlueOS DVL extension sends. ArduSub's EKF3 navigates on it
   (EK3_SRC1_POSXY/VELXY = ExternalNav) plus baro depth and compass. So the estimate drifts like a real ROV's.
2. Survey plan route_dam.sub_plan() flown in GUIDED with SET_POSITION_TARGET_GLOBAL_INT (position + yaw).
3. Cooperation: the drone's state arrives on UDP :15002 (mav_bridge.py). When the drone passes a defect at the water
   line (z < 1.6 m, within 5 m of the drone), the bridge queues an ROV task to inspect the wall directly below it.
4. Streams truth pose, EKF estimate, DVL, plan item and tasks as JSON to Isaac on UDP :15001.
"""
import json, math, os, socket, sys, time
import numpy as np
from pymavlink import mavutil
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import route_dam as RT

H = os.path.dirname(os.path.abspath(__file__))
SURF_AMSL = 2.0                                                  # ArduSub home altitude = the water surface
HOME = [float(v) for v in open(os.path.join(H, "..", "sitl", "home_sub.txt")).read().strip().split(",")]
PLAN = RT.sub_plan()
GT = json.load(open(os.path.join(H, "..", "data", "defects_gt.json")))["defects"]
WATERLINE = [d for d in GT if d["position"][2] < 1.6 and d["position"][2] > -0.2]
rng = np.random.default_rng(11)

out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
coop = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); coop.bind(("127.0.0.1", 15002)); coop.setblocking(False)
m = mavutil.mavlink_connection("tcp:127.0.0.1:5770", source_system=250, source_component=191)


def autopilot_hb(timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return hb
    sys.exit("no ArduSub heartbeat")


def cmd(c, *p):
    p = list(p) + [0] * (7 - len(p))
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *p)


def world_of(lat_e7, lon_e7, z):
    w = RT.geo_to_world(lat_e7 / 1e7, lon_e7 / 1e7); w[2] = z
    return w


def geo_of(p):
    return int((RT.LAT0 + p[1] / 111320.0) * 1e7), int((RT.LON0 + p[0] / RT._K) * 1e7)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- simulated DVL (Water Linked A50 class)
class DVL:
    SCALE_ERR, NOISE = 0.004, 0.003          # 0.4 % scale factor error, 3 mm/s white noise per axis
    ALT_MIN, ALT_MAX = 0.05, 50.0

    def __init__(self):
        self.prev = None; self.scale = 1.0 + rng.normal(0, self.SCALE_ERR, 2); self.vel = np.zeros(3); self.alt = float("nan"); self.lock = False

    def update(self, t, pos, rpy):
        """pos: true world position, rpy: true roll/pitch/yaw (NED yaw, rad). Sends VISION_POSITION_DELTA at the call rate."""
        if self.prev is None:
            self.prev = (t, pos.copy(), np.array(rpy)); return
        t0, p0, a0 = self.prev; dt = t - t0
        if dt < 0.08: return
        self.prev = (t, pos.copy(), np.array(rpy))
        dE, dN, dU = pos - p0
        yaw = a0[2]
        fwd = math.cos(yaw) * dN + math.sin(yaw) * dE; right = -math.sin(yaw) * dN + math.cos(yaw) * dE
        self.alt = pos[2] - RT.bed_z(pos[0], pos[1])
        self.lock = bool(self.ALT_MIN < self.alt < self.ALT_MAX)
        d = np.array([fwd * self.scale[0], right * self.scale[1], -dU]) + rng.normal(0, self.NOISE * dt, 3)
        self.vel = d / dt
        dang = [wrap(rpy[0] - a0[0]), wrap(rpy[1] - a0[1]), wrap(rpy[2] - a0[2])]
        m.mav.vision_position_delta_send(int(t * 1e6), int(dt * 1e6), dang, d.tolist() if self.lock else [0.0, 0.0, 0.0],
                                         100.0 if self.lock else 0.0)


# ---------------------------------------------------------------- startup
hb = autopilot_hb(); print("ARDUSUB sys", m.target_system, flush=True)
for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE, 25), (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 25), (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)):
    cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, int(1e6 / hz))
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
m.mav.set_gps_global_origin_send(m.target_system, int(HOME[0] * 1e7), int(HOME[1] * 1e7), int(SURF_AMSL * 1000))
dvl = DVL()
S = dict(t=0.0, item=0, label="boot", tasks=[], flagged=[], mode="?", armed=False)
truth, rpy, est = None, (0.0, 0.0, 0.0), None


def pump(timeout=0.05):
    """read MAVLink, run the DVL, forward drone flags, stream state to Isaac."""
    global truth, rpy, est
    t_end = time.time() + timeout
    while True:
        r = m.recv_match(blocking=True, timeout=max(t_end - time.time(), 0.001))
        if r is None:
            break
        tp = r.get_type()
        if tp == "SIMSTATE":
            z = est[2] if est is not None else 0.0                                      # depth: baro (ArduSub's own)
            truth = world_of(r.lat, r.lng, z); rpy = (r.roll, r.pitch, r.yaw)
            dvl.update(time.time(), truth, rpy)
        elif tp == "GLOBAL_POSITION_INT" and (r.lat or r.lon):                         # 0,0 until the EKF has an origin
            est = world_of(r.lat, r.lon, r.alt / 1000.0 - SURF_AMSL)
        elif tp == "HEARTBEAT" and r.get_srcSystem() == m.target_system:
            S["mode"] = mavutil.mode_string_v10(r); S["armed"] = bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif tp == "STATUSTEXT":
            print("SUB:", r.text, flush=True)
        if time.time() > t_end:
            break
    try:                                                                                # drone state -> waterline defects it has seen
        while True:
            dr = json.loads(coop.recv(65536))
            if "pos" in dr:
                dp = np.array(dr["pos"])
                for d in WATERLINE:
                    if d["id"] not in S["flagged"] and np.linalg.norm(dp - np.array(d["position"])) < 5.0:
                        S["flagged"].append(d["id"]); S["tasks"].append(d["id"]); QUEUE.append(RT.waterline_task(d))
                        print(f"COOP drone flagged {d['id']} ({d['type']} at z={d['position'][2]:.1f} m) -> ROV task queued", flush=True)
    except BlockingIOError:
        pass
    if truth is not None:
        S.update(t=time.time(), pos=truth.tolist(), rpy=list(rpy), est=(est.tolist() if est is not None else None),
                 dvl=dict(vel=dvl.vel.tolist(), alt=float(dvl.alt), lock=bool(dvl.lock)))
        out.sendto(json.dumps(S).encode(), ("127.0.0.1", 15001))


def goto(p, yaw_deg):
    lat, lon = geo_of(p); yaw_ned = math.radians(90.0 - yaw_deg)
    m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                                              0b100111111000, lat, lon, SURF_AMSL + float(p[2]), 0, 0, 0, 0, 0, 0, yaw_ned, 0)


QUEUE = []
t0 = time.time()
while truth is None or est is None:
    if int((time.time() - t0) * 5) % 10 == 0:                                         # the EKF ignores the origin until it is ready
        m.mav.set_gps_global_origin_send(m.target_system, int(HOME[0] * 1e7), int(HOME[1] * 1e7), int(SURF_AMSL * 1000))
    pump(0.2)
    if time.time() - t0 > 60: sys.exit("no SIMSTATE / position: is the EKF origin set?")
print("DVL online, EKF has a position", flush=True)
for _ in range(40):                                                                     # let EKF3 converge on the DVL
    pump(0.25)
cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4)                                          # GUIDED (custom mode 4)
for i in range(20):
    pump(0.5)
    if S["mode"] == "GUIDED": break
    cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4)
for i in range(20):
    cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    pump(1.0)
    if S["armed"]: break
print("ARMED" if S["armed"] else "ARM FAILED", S["mode"], flush=True)
if not S["armed"]: sys.exit(1)
print("DIVING", flush=True)

# ---------------------------------------------------------------- survey + cooperative tasks
plan = list(PLAN)
while plan or QUEUE:
    if QUEUE and plan and plan[0][3] not in ("dive",):                                  # drone-flagged tasks pre-empt the survey
        item = QUEUE.pop(0); from_queue = True
    else:
        item = plan.pop(0); from_queue = False
    p, yaw, hold, label = item
    S["item"] += 1; S["label"] = label; S["target"] = [float(v) for v in p]
    print(f"ITEM {S['item']} {label} -> ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})", flush=True)
    t_start, reached, t_sent = time.time(), None, 0.0
    while True:
        if time.time() - t_sent > 5.0:                                                  # a new target restarts ArduSub's trajectory
            goto(p, yaw); t_sent = time.time()                                          # from rest, so send it once, refresh rarely
        pump(0.1)
        e = np.linalg.norm(np.array(est) - p) if est is not None else 99
        if reached is None and (e < 0.45 or time.time() - t_start > 120):
            reached = time.time()
        if reached is not None and time.time() - reached >= min(hold, 1e6):
            break
        if label == "surface" and reached is not None and time.time() - reached > 5:
            break
    if from_queue:
        S["tasks"] = [t for t in S["tasks"] if t not in label]
        print(f"TASK DONE {label}", flush=True)
print("SURVEY COMPLETE", flush=True)
cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
for _ in range(1000000):
    pump(0.1)
