#!/usr/bin/env bash
#
# run_live.sh - launch the full bridge-inspection drone stack, in order.
#
#   (a) ArduCopter SITL           (listens on tcp 5760, QGC port tcp 5762)
#   (b) Isaac Sim bridge world    (binds UDP 127.0.0.1:15000)
#   (c) MAVLink mission bridge    (prints "FLYING" when the vehicle is airborne)
#   (d) QGroundControl            (auto-connects to tcp 5762)
#
# Each step waits until the previous one is actually ready before the next
# starts. Logs land in logs/, and the PID of every process we start is written
# (one "name=pid" per line) to logs/pids so stop_live.sh can kill exactly those.
#
# This script does NOT kill anything. It refuses to start a second ArduCopter.

set -u

# --- paths -------------------------------------------------------------------
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGS="$ROOT/logs"
PIDS="$LOGS/pids"
PY="$HOME/IsaacLab/env_isaaclab/bin/python"
SITL_LAUNCHER="$ROOT/sitl/launch.sh"
QGC="$HOME/Applications/QGroundControl-x86_64.AppImage"

mkdir -p "$LOGS"
: > "$PIDS"

log()  { printf '[run_live] %s\n' "$*"; }
die()  { printf '[run_live] ERROR: %s\n' "$*" >&2; exit 1; }

record_pid() {
    # record_pid <name> <pid>
    printf '%s=%s\n' "$1" "$2" >> "$PIDS"
    log "started $1 (pid $2)"
}

# Wait until a TCP port is in LISTEN state (ss -lnt), up to <timeout> seconds.
wait_tcp_listen() {
    local port="$1" timeout="${2:-120}" i=0
    while (( i < timeout )); do
        if ss -lnt 2>/dev/null | grep -Eq "[:.]${port}[[:space:]]"; then
            return 0
        fi
        sleep 1; (( i++ ))
    done
    return 1
}

# Wait until a UDP port is bound (ss -lun), up to <timeout> seconds.
wait_udp_bound() {
    local port="$1" timeout="${2:-300}" i=0
    while (( i < timeout )); do
        if ss -lun 2>/dev/null | grep -Eq "[:.]${port}[[:space:]]"; then
            return 0
        fi
        sleep 1; (( i++ ))
    done
    return 1
}

# Wait until <pattern> appears in <logfile>, while <pid> is still alive.
wait_log_line() {
    local logfile="$1" pattern="$2" pid="$3" timeout="${4:-300}" i=0
    while (( i < timeout )); do
        if grep -q -- "$pattern" "$logfile" 2>/dev/null; then
            return 0
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            return 2   # process died before printing the line
        fi
        sleep 1; (( i++ ))
    done
    return 1
}

# -----------------------------------------------------------------------------
# (a) ArduCopter SITL
# -----------------------------------------------------------------------------
# Refuse to start a second autopilot: a live sim may already be running.
if pgrep -x arducopter >/dev/null 2>&1; then
    die "an arducopter process is already running - refusing to start a second one."
fi

log "launching ArduCopter SITL ..."
# SITL wants a PTY on stdin; 'script' gives it one, 'setsid' detaches it so it
# survives this shell. Output is tee'd by script into the SITL log.
setsid script -qfc "$SITL_LAUNCHER" "$LOGS/sitl.log" >/dev/null 2>&1 &
SITL_WRAP_PID=$!
record_pid sitl "$SITL_WRAP_PID"

log "waiting for SITL to listen on tcp 5760 ..."
if ! wait_tcp_listen 5760 120; then
    die "SITL did not start listening on tcp 5760 (see $LOGS/sitl.log)."
fi
log "SITL is up (tcp 5760 listening; QGC port tcp 5762)."
# Record the real arducopter PID too (child of the 'script' wrapper) so that
# stop_live.sh can terminate the autopilot itself, not just the PTY wrapper.
# pgrep -x matches the exact process name (never pkill -f).
AC_PID="$(pgrep -x arducopter | head -n1)"
if [[ -n "${AC_PID:-}" ]]; then
    record_pid arducopter "$AC_PID"
fi

# (a2) ArduSub SITL for the BlueROV2 Heavy (instance 1: tcp 5770 bridge, 5772 QGC)
if [[ "${SUB:-1}" == "1" ]]; then
    if pgrep -x ardusub >/dev/null 2>&1; then die "an ardusub process is already running - refusing to start a second one."; fi
    rm -f "$ROOT/sitl/run_sub/eeprom.bin"
    log "launching ArduSub SITL (BlueROV2 Heavy, DVL navigation) ..."
    setsid script -qfc "$ROOT/sitl/launch_sub.sh" "$LOGS/sitl_sub.log" >/dev/null 2>&1 &
    record_pid sitl_sub "$!"
    wait_tcp_listen 5770 120 || die "ArduSub did not listen on tcp 5770 (see $LOGS/sitl_sub.log)."
    record_pid ardusub "$(pgrep -x ardusub | head -n1)"
fi

# -----------------------------------------------------------------------------
# (b) Isaac Sim bridge world
# -----------------------------------------------------------------------------
log "launching Isaac Sim Charles River Dam world (publishing ROS 2 topics) ..."
set +u; source /opt/ros/jazzy/setup.bash; set -u
MAV=1 OMNI_KIT_ACCEPT_EULA=YES "$PY" "$ROOT/sim/charles_dam_world.py" \
    >"$LOGS/bridge_world.log" 2>&1 &
WORLD_PID=$!
record_pid bridge_world "$WORLD_PID"

log "waiting for the Isaac world to bind UDP 127.0.0.1:15000 ..."
if ! wait_udp_bound 15000 600; then
    die "Isaac world did not bind UDP 15000 (see $LOGS/bridge_world.log)."
fi
log "Isaac world is listening on UDP 15000."

# -----------------------------------------------------------------------------
# (c) MAVLink mission bridge
# -----------------------------------------------------------------------------
log "launching the MAVLink mission bridge ..."
ROUTE=route_dam "$PY" "$ROOT/sim/mav_bridge.py" >"$LOGS/mav_bridge.log" 2>&1 &
BRIDGE_PID=$!
record_pid mav_bridge "$BRIDGE_PID"

if [[ "${SUB:-1}" == "1" ]]; then
    log "launching the ROV bridge (DVL + survey + drone->ROV tasking) ..."
    "$PY" "$ROOT/sim/sub_bridge.py" >"$LOGS/sub_bridge.log" 2>&1 &
    record_pid sub_bridge "$!"
fi

log "waiting for the bridge to report FLYING ..."
wait_log_line "$LOGS/mav_bridge.log" "FLYING" "$BRIDGE_PID" 600
case $? in
    0) log "mission bridge is FLYING." ;;
    2) die "mav_bridge exited before takeoff (see $LOGS/mav_bridge.log)." ;;
    *) die "mav_bridge did not reach FLYING in time (see $LOGS/mav_bridge.log)." ;;
esac

# -----------------------------------------------------------------------------
# (d) QGroundControl
# -----------------------------------------------------------------------------
# QGC: make sure it auto-connects to the BlueROV2 too (ArduSub sysid 2 on tcp 5772). QGC rewrites its ini on exit,
# so this runs before it starts.
python3 - <<'PYEOF'
import os, re
p = os.path.expanduser("~/.config/QGroundControl/QGroundControl.ini")
if os.path.exists(p):
    s = open(p).read()
    if "port=5772" not in s and "[LinkConfigurations]" in s:
        n = int(re.search(r"\[LinkConfigurations\][^\[]*?count=(\d+)", s, re.S).group(1))
        add = (f"Link{n}\\auto=true\nLink{n}\\high_latency=false\nLink{n}\\host=127.0.0.1\n"
               f"Link{n}\\name=BlueROV2 ArduSub SITL\nLink{n}\\port=5772\nLink{n}\\type=2\n")
        s = re.sub(r"(\[LinkConfigurations\][^\[]*?)count=\d+", lambda m_: m_.group(1) + add + f"count={n + 1}", s, count=1, flags=re.S)
        open(p + ".bak-rov", "w").write(open(p).read()); open(p, "w").write(s)
        print("[run_live] QGC: added the BlueROV2 link (tcp 5772)")
PYEOF
if [[ -x "$QGC" ]]; then
    log "launching QGroundControl (auto-connects to tcp 5762) ..."
    setsid "$QGC" >"$LOGS/qgc.log" 2>&1 &
    QGC_PID=$!
    record_pid qgc "$QGC_PID"
else
    log "QGroundControl not found/executable at $QGC - skipping (stack is up)."
fi

# -----------------------------------------------------------------------------
# (e) RViz2: map, odom, flown path, mission, lidar, RGB + thermal
# -----------------------------------------------------------------------------
log "launching RViz2 ..."
setsid "$ROOT/scripts/rviz.sh" >"$LOGS/rviz.log" 2>&1 &
record_pid rviz "$!"

log "stack is up. PIDs recorded in $PIDS"
log "stop it with: $ROOT/scripts/stop_live.sh"
