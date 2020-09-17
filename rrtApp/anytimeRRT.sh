#!/bin/sh
# This is a comment!
roscore || true
./build-simplePaint-Desktop-Debug/simplePaint anytimerrt
roslaunch anytimerrt anytimeRRT.launch
exit 0