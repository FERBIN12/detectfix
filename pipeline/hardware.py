"""Hardware health monitoring (dashboard tab 2) -- SIMULATED telemetry.

The recorded bags carry no battery, motor/ESC, thermal, link or leak telemetry,
so this module generates a plausible mock of it for both platforms. Every block
is marked `is_simulated=True` and the dashboard labels it as such; it is never
mixed into the real health numbers in health.py.

Code enforces: the mock values are deterministic (seeded by mission id), and
every component status (ok / warn / fault) is decided by the named thresholds
below, never by a model. Swap `simulate()` for a real MAVLink / BlueOS reader
and the thresholds + dashboard stay the same.
"""
from __future__ import annotations

import hashlib
import random

# --- thresholds (status rules) -------------------------------------------------
BATTERY_WARN_PCT = 30.0         # land / surface soon
BATTERY_FAULT_PCT = 15.0        # land / surface now
CELL_IMBALANCE_WARN_V = 0.05    # max-min cell voltage spread
CELL_IMBALANCE_FAULT_V = 0.10
MOTOR_TEMP_WARN_C = 70.0        # ESC / motor case temperature
MOTOR_TEMP_FAULT_C = 85.0
CPU_TEMP_WARN_C = 75.0          # companion computer
CPU_TEMP_FAULT_C = 90.0
LINK_RSSI_WARN_DBM = -80.0      # radio (drone) / tether modem (ROV)
LINK_RSSI_FAULT_DBM = -90.0
LINK_LATENCY_WARN_MS = 150.0
STORAGE_WARN_PCT = 85.0
VIBRATION_WARN_MS2 = 30.0       # IMU vibration (ArduPilot VIBE, m/s^2)
VIBRATION_FAULT_MS2 = 60.0
LEAK_FAULT = True               # any water in an ROV enclosure is a fault

STATUS_RANK = {"ok": 0, "warn": 1, "fault": 2}


def _status_hi(value, warn, fault):
    return "fault" if value >= fault else "warn" if value >= warn else "ok"


def _status_lo(value, warn, fault):
    return "fault" if value <= fault else "warn" if value <= warn else "ok"


def _rng(mission_id: str, platform: str) -> random.Random:
    seed = int(hashlib.sha256(f"{mission_id}:{platform}".encode()).hexdigest()[:12], 16)
    return random.Random(seed)


def _battery(rng, duration_s, cells, cap_mah, start_pct, end_pct, n=40):
    """Discharge curve: % and pack voltage over the mission."""
    series = []
    for i in range(n):
        f = i / (n - 1)
        pct = start_pct - (start_pct - end_pct) * (f ** 1.1) + rng.uniform(-0.4, 0.4)
        cell_v = 3.5 + 0.7 * (pct / 100.0) ** 0.8          # ~4.2 V full, ~3.5 V empty
        series.append({"t_s": round(f * duration_s, 1), "pct": round(max(pct, 0), 1),
                       "voltage_v": round(cell_v * cells, 2)})
    last = series[-1]
    cell_vs = [round(last["voltage_v"] / cells + rng.uniform(-0.02, 0.02), 3) for _ in range(cells)]
    spread = round(max(cell_vs) - min(cell_vs), 3)
    status = max(_status_lo(last["pct"], BATTERY_WARN_PCT, BATTERY_FAULT_PCT),
                 _status_hi(spread, CELL_IMBALANCE_WARN_V, CELL_IMBALANCE_FAULT_V), key=STATUS_RANK.get)
    used = round(cap_mah * (series[0]["pct"] - last["pct"]) / 100)
    return {"cells": cells, "capacity_mah": cap_mah, "now_pct": last["pct"], "now_voltage_v": last["voltage_v"],
            "cell_voltages_v": cell_vs, "cell_spread_v": spread, "used_mah": used,
            "series": series, "status": status}


def _motors(rng, names, base_temp, base_rpm, hot_index):
    out = []
    for i, name in enumerate(names):
        temp = base_temp + rng.uniform(-3, 3) + (16 if i == hot_index else 0)
        rpm = base_rpm * rng.uniform(0.95, 1.05) * (1.06 if i == hot_index else 1)
        cur = base_rpm / 400 * rng.uniform(0.9, 1.1) * (1.15 if i == hot_index else 1)
        out.append({"name": name, "temp_c": round(temp, 1), "rpm": int(rpm), "current_a": round(cur, 1),
                    "status": _status_hi(temp, MOTOR_TEMP_WARN_C, MOTOR_TEMP_FAULT_C)})
    return out


def _component(name, status, value, note):
    return {"name": name, "status": status, "value": value, "note": note}


def drone_hardware(mission_id: str, duration_s: float) -> dict:
    rng = _rng(mission_id, "drone")
    duration_s = duration_s or 420.0
    bat = _battery(rng, duration_s, cells=6, cap_mah=5200, start_pct=100.0, end_pct=rng.uniform(38, 46))
    motors = _motors(rng, ["M1 front-right", "M2 rear-left", "M3 front-left", "M4 rear-right"],
                     base_temp=52.0, base_rpm=6200, hot_index=rng.randrange(4))
    cpu = round(rng.uniform(58, 66), 1)
    rssi = round(rng.uniform(-71, -64), 1)
    lat = round(rng.uniform(28, 55), 1)
    vibe = round(rng.uniform(14, 24), 1)
    storage = round(rng.uniform(41, 58), 1)
    comps = [
        _component("Flight controller (Pixhawk 6C)", "ok", "EKF healthy", "attitude + position estimates converged"),
        _component("Companion computer", _status_hi(cpu, CPU_TEMP_WARN_C, CPU_TEMP_FAULT_C), f"{cpu} °C",
                   "records the bag, runs the onboard pre-filter"),
        _component("Ouster OS0-128 LiDAR", "ok", "streaming", "~10 Hz, 128 beams"),
        _component("Gimbal camera (CGO3)", "ok", "streaming", "960x540 RGB + thermal"),
        _component("GNSS (RTK)", "ok", "RTK fixed", "see GNSS & positioning above"),
        _component("Telemetry radio", _status_lo(rssi, LINK_RSSI_WARN_DBM, LINK_RSSI_FAULT_DBM),
                   f"{rssi} dBm / {lat} ms", "link to the ground station"),
        _component("IMU vibration", _status_hi(vibe, VIBRATION_WARN_MS2, VIBRATION_FAULT_MS2),
                   f"{vibe} m/s²", "high vibration degrades LiDAR registration and imagery"),
        _component("Onboard storage", _status_hi(storage, STORAGE_WARN_PCT, 101), f"{storage}% used",
                   "bag + frames for this mission"),
    ]
    return _assemble("drone", "Holybro X500 V2", bat, motors, "Motors / ESCs", comps)


def rov_hardware(mission_id: str, duration_s: float) -> dict:
    rng = _rng(mission_id, "rov")
    duration_s = duration_s or 420.0
    bat = _battery(rng, duration_s, cells=4, cap_mah=18000, start_pct=100.0, end_pct=rng.uniform(62, 72))
    motors = _motors(rng, [f"T{i} {p}" for i, p in enumerate(
        ["vert-FL", "vert-FR", "vert-RL", "vert-RR", "horiz-FL", "horiz-FR", "horiz-RL", "horiz-RR"], 1)],
        base_temp=31.0, base_rpm=2400, hot_index=rng.randrange(8))
    cpu = round(rng.uniform(55, 63), 1)
    rssi = round(rng.uniform(-66, -58), 1)
    lat = round(rng.uniform(12, 30), 1)
    leak = False
    internal_temp = round(rng.uniform(24, 31), 1)
    humidity = round(rng.uniform(28, 41), 1)
    comps = [
        _component("Autopilot (Navigator / ArduSub)", "ok", "EKF healthy", "DVL-aided navigation"),
        _component("Companion computer (BlueOS)", _status_hi(cpu, CPU_TEMP_WARN_C, CPU_TEMP_FAULT_C),
                   f"{cpu} °C", "records sonar + camera"),
        _component("Multibeam imaging sonar", "ok", "streaming", "130° fan, 12 m range"),
        _component("DVL (A50-class)", "ok", "bottom lock", "see DVL lock above"),
        _component("Camera + lights", "ok", "streaming", "640x360; turbidity limits use"),
        _component("Tether / Fathom-X modem", _status_lo(rssi, LINK_RSSI_WARN_DBM, LINK_RSSI_FAULT_DBM),
                   f"{rssi} dBm / {lat} ms", "only link to the surface"),
        _component("Leak sensors (4)", "fault" if leak and LEAK_FAULT else "ok", "dry" if not leak else "WATER",
                   "any water in an enclosure aborts the dive"),
        _component("Electronics enclosure", "warn" if humidity > 60 else "ok",
                   f"{internal_temp} °C, {humidity}% RH", "humidity rising signals a slow leak"),
    ]
    return _assemble("rov", "BlueROV2 Heavy", bat, motors, "Thrusters", comps)


def _assemble(platform, model, bat, motors, motor_label, comps):
    comps = [_component("Battery", bat["status"], f"{bat['now_pct']}% · {bat['now_voltage_v']} V",
                        f"{bat['cells']}S {bat['capacity_mah']} mAh, cell spread {bat['cell_spread_v']} V")] + comps
    alerts = []
    for c in comps:
        if c["status"] != "ok":
            alerts.append({"level": c["status"], "text": f"{c['name']}: {c['value']} — {c['note']}"})
    for m in motors:
        if m["status"] != "ok":
            alerts.append({"level": m["status"],
                           "text": f"{m['name']} running hot: {m['temp_c']} °C "
                                   f"(warn ≥ {MOTOR_TEMP_WARN_C:g} °C). Inspect for fouling or bearing wear."})
    overall = max([c["status"] for c in comps] + [m["status"] for m in motors], key=STATUS_RANK.get)
    return {"platform": platform, "model": model, "is_simulated": True, "overall": overall,
            "battery": bat, "motors": motors, "motor_label": motor_label,
            "components": comps, "alerts": alerts,
            "thresholds": {"battery_warn_pct": BATTERY_WARN_PCT, "motor_temp_warn_c": MOTOR_TEMP_WARN_C,
                           "motor_temp_fault_c": MOTOR_TEMP_FAULT_C, "rssi_warn_dbm": LINK_RSSI_WARN_DBM}}


def simulate(mission_id: str, drone_duration_s: float = 0.0, rov_duration_s: float = 0.0,
             rov_flown: bool = True) -> dict:
    """Mock hardware telemetry for both platforms. The ROV block is omitted
    (None) on drone-only missions -- there was no vehicle to monitor."""
    return {"drone": drone_hardware(mission_id, drone_duration_s),
            "rov": rov_hardware(mission_id, rov_duration_s) if rov_flown else None}
