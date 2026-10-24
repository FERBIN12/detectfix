#!/bin/bash
# ArduCopter SITL. HOME_FILE picks the launch point (default home_dam.txt = Craigie canal RIB).
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$D/run" && cd "$D/run"
HOME_FILE="${HOME_FILE:-$D/home_dam.txt}"
AP="${ARDUPILOT:-$HOME/rov_course/third_party/ardupilot}"     # ArduPilot checkout with build/sitl (./waf copter)
exec $AP/build/sitl/bin/arducopter --model quad --speedup 1 --home $(cat "$HOME_FILE"),2.4,0 --defaults $AP/Tools/autotest/default_params/copter.parm -I0
