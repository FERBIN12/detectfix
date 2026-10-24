#!/usr/bin/env bash
#
# record_run.sh - one full inspection flight recorded to a rosbag (MCAP, zstd).
#
#   scripts/record_run.sh            # bag lands in bags/run_<time>/
#
# Starts the recorder, then run_live.sh, waits until ArduCopter disarms after the mission (or MAX_S seconds),
# stops the recorder cleanly (SIGTERM: background jobs start with SIGINT ignored), then stop_live.sh. /map is not recorded: rebuild it from /lidar/points + /tf.

set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAX_S="${MAX_S:-900}"
OUT="$ROOT/bags/run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROOT/bags" "$ROOT/logs"
set +u; source /opt/ros/jazzy/setup.bash; set -u

TOPICS=(/tf /tf_static /odom /path /mission /mission/markers /defects /drone/status /lidar/points /camera/rgb /camera/thermal)
echo "[record] bag -> $OUT"
setsid ros2 bag record -s mcap --storage-preset-profile zstd_fast -o "$OUT" "${TOPICS[@]}" > "$ROOT/logs/bag.log" 2>&1 &
BAG_PID=$!

"$ROOT/scripts/run_live.sh" || { kill -TERM "$BAG_PID"; exit 1; }

echo "[record] flying; waiting for the mission to end (disarm) ..."
t0=$(date +%s)
while (( $(date +%s) - t0 < MAX_S )); do
    if grep -qiE "disarm" "$ROOT/logs/mav_bridge.log"; then echo "[record] disarmed after the mission"; break; fi
    sleep 2
done
sleep 3
kill -TERM "$BAG_PID"; wait "$BAG_PID" 2>/dev/null
"$ROOT/scripts/stop_live.sh"
cp "$ROOT/data/defects_gt.json" "$OUT/"
ros2 bag info "$OUT" | grep -E "Duration|Messages|Topic:|Count" | head -20
du -sh "$OUT"
