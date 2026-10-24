#!/bin/bash
# ArduSub SITL (BlueROV2 Heavy: vectored_6dof frame), instance 1 -> tcp 5770 (bridge) / 5772 (QGC). Home = the RIB at the surface.
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$D/run_sub" && cd "$D/run_sub"
HOME_FILE="${HOME_FILE:-$D/home_sub.txt}"
AP="${ARDUPILOT:-$HOME/rov_course/third_party/ardupilot}"
exec $AP/build/sitl/bin/ardusub --model vectored_6dof --speedup 1 --home $(cat "$HOME_FILE"),2.0,0 --defaults $AP/Tools/autotest/default_params/sub-6dof.parm,$D/sub_dvl.parm -I1
