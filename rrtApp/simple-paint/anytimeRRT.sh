#!/bin/sh
# This is a comment!
roscore || true
./simplePaint anytimerrt
roslaunch anytimerrt anytimeRRT.launch
exit 0