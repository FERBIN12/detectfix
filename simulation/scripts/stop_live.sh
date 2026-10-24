#!/usr/bin/env bash
#
# stop_live.sh - stop the bridge-inspection stack started by run_live.sh.
#
# Kills EXACTLY the PIDs recorded in logs/pids (one "name=pid" per line) and
# nothing else. It never uses pkill -f / pattern matching, so it cannot touch
# an unrelated process that happens to share a command line.
#
# Order: reverse of launch (qgc -> mav_bridge -> bridge_world -> arducopter/sitl),
# SIGTERM first, then SIGKILL for anything that refuses to exit.

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS="$ROOT/logs/pids"

log() { printf '[stop_live] %s\n' "$*"; }

if [[ ! -f "$PIDS" ]]; then
    log "no PID file at $PIDS - nothing to stop."
    exit 0
fi

# Read "name=pid" lines into parallel arrays, preserving launch order.
names=(); pids=()
while IFS='=' read -r name pid; do
    [[ -z "${name:-}" || -z "${pid:-}" ]] && continue
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    names+=("$name"); pids+=("$pid")
done < "$PIDS"

n=${#pids[@]}
if (( n == 0 )); then
    log "PID file is empty - nothing to stop."
    exit 0
fi

# SIGTERM in reverse launch order.
for (( i = n - 1; i >= 0; i-- )); do
    name="${names[$i]}"; pid="${pids[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
        log "SIGTERM $name (pid $pid)"
        kill -TERM "$pid" 2>/dev/null
    else
        log "$name (pid $pid) is not running"
    fi
done

# Give them a few seconds to exit cleanly.
sleep 5

# SIGKILL anything still alive.
for (( i = n - 1; i >= 0; i-- )); do
    name="${names[$i]}"; pid="${pids[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
        log "SIGKILL $name (pid $pid)"
        kill -KILL "$pid" 2>/dev/null
    fi
done

log "done. Removing $PIDS"
rm -f "$PIDS"
