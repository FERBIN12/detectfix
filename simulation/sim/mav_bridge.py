"""MAVLink bridge: ArduCopter SITL <-> inspection mission <-> Isaac Sim.

    ~/IsaacLab/env_isaaclab/bin/python mav_bridge.py        (SITL must be listening on tcp:5760; QGC uses tcp:5762)

1. Latches the AUTOPILOT heartbeat (not a GCS/probe one), sets inspection flight params.
2. Uploads the bridge inspection mission (route.flight_plan) as MAVLink mission items, so QGroundControl shows it.
3. GUIDED -> arm -> takeoff -> AUTO, then RTL at the end of the mission.
4. Streams the vehicle state (pose, attitude, GPS, mode, mission item) as JSON over UDP to Isaac (127.0.0.1:15000).
5. GNSS under the deck: while the vehicle is under the bridge it lowers SITL's satellite count (SIM_GPS1_NUMSATS), so
   the autopilot and QGC really see degraded GPS. That is the real reason bridge drones need lidar/visual odometry.
"""
import json, math, socket, sys, time
import numpy as np
from pymavlink import mavutil
import importlib, os
_RT = importlib.import_module(os.environ.get("ROUTE", "route"))
flight_plan, to_geo, ned_to_world = _RT.flight_plan, _RT.to_geo, _RT.ned_to_world

PLAN = flight_plan()
out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=250, source_component=191)


def autopilot_hb(timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return hb
    sys.exit("no autopilot heartbeat")


def setp(name, val):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < 3:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if r and r.param_id.strip("\x00") == name:
            return True
    print("param not confirmed:", name, flush=True)
    return False


def cmd(c, *p):
    p = list(p) + [0] * (7 - len(p))
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *p)


def upload_mission():
    items = []
    seq = 0
    def add(command, x=0, y=0, z=0, p1=0, p2=0, p3=0, p4=0, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT):
        nonlocal seq
        items.append(mavutil.mavlink.MAVLink_mission_item_int_message(m.target_system, m.target_component, seq, frame, command,
                                                                        0, 1, p1, p2, p3, p4, x, y, z))
        seq += 1
    lat, lon, _ = to_geo(PLAN[0][0])
    add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, int(lat * 1e7), int(lon * 1e7), 0)          # item 0 = home
    for pos, tgt, el in PLAN:
        lat, lon, alt = to_geo(pos)
        add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, int(lat * 1e7), int(lon * 1e7), alt, p2=0.6)   # 0.6 m acceptance
    add(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, frame=mavutil.mavlink.MAV_FRAME_MISSION)
    m.mav.mission_clear_all_send(m.target_system, m.target_component); time.sleep(0.5)
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = set()
    while len(sent) < len(items):
        r = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=5)
        if r is None:
            sys.exit(f"mission upload stalled at {len(sent)}/{len(items)}")
        m.mav.send(items[r.seq]); sent.add(r.seq)
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    print("MISSION", len(items), "items, ack", ack.type if ack else None, flush=True)


def mode(name):
    m.set_mode(m.mode_mapping()[name])
    t0 = time.time()
    while time.time() - t0 < 5:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.get_srcSystem() == m.target_system and mavutil.mode_string_v10(hb) == name:
            return True
    return False


hb = autopilot_hb(); print("AUTOPILOT sys", m.target_system, flush=True)
GPS_SATS_PARAM = "SIM_GPS1_NUMSATS" if setp("SIM_GPS1_NUMSATS", 12) else "SIM_GPS_NUMSATS"   # name differs between ArduPilot versions
for k, v in dict(WPNAV_SPEED=200, WPNAV_SPEED_DN=100, WPNAV_SPEED_UP=150, WPNAV_ACCEL=150, WPNAV_RADIUS=60,
                 WP_YAW_BEHAVIOR=0, RTL_ALT=600, FENCE_ENABLE=0, ARMING_CHECK=0).items():
    setp(k, v)
for msg, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30), (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30),
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5), (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 5)):
    cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg, 1e6 / hz)
# wait for EKF/GPS before arming
t0 = time.time()
while time.time() - t0 < 90:
    g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
    if g and g.fix_type >= 3:
        break
time.sleep(8)
upload_mission()
def armed():
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    return bool(hb and hb.get_srcSystem() == m.target_system and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def rel_alt():
    p = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
    return -p.z if p else 0.0


mode("GUIDED")
for attempt in range(60):                                                   # EKF needs a position estimate before arming
    cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if any(armed() for _ in range(3)):
        break
    time.sleep(2)
print("ARMED", flush=True)
cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 5.6)
t0 = time.time()
while rel_alt() < 4.5 and time.time() - t0 < 30:
    pass
mode("AUTO"); cmd(mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
print("FLYING", flush=True)

state = dict(t=0)
sats_set = None
while True:
    r = m.recv_match(blocking=True, timeout=1)
    if r is None:
        continue
    tp = r.get_type()
    if tp == "LOCAL_POSITION_NED":
        p = ned_to_world(r.x, r.y, r.z); state.update(pos=p.tolist(), vel=[r.vy, r.vx, -r.vz])
        under = _RT.under_deck(p) if hasattr(_RT, "under_deck") else False
        want = 7 if under else 12                                        # SITL default sats 10-12; under the deck: 7
        if want != sats_set:
            setp(GPS_SATS_PARAM, want); sats_set = want
    elif tp == "ATTITUDE":
        state.update(roll=r.roll, pitch=r.pitch, yaw=r.yaw)
    elif tp == "GPS_RAW_INT":
        state.update(sats=r.satellites_visible, hdop=r.eph / 100.0, fix=r.fix_type)
    elif tp == "MISSION_CURRENT":
        state.update(wp=r.seq)
    elif tp == "HEARTBEAT" and r.get_srcSystem() == m.target_system:
        state.update(mode=mavutil.mode_string_v10(r), armed=bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
    elif tp == "STATUSTEXT":
        print("AP:", r.text, flush=True)
    state["t"] = time.time()
    if tp in ("LOCAL_POSITION_NED", "ATTITUDE"):
        msg = json.dumps(state).encode()
        out.sendto(msg, ("127.0.0.1", 15000)); out.sendto(msg, ("127.0.0.1", 15002))   # Isaac + the ROV coordinator
