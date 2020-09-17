#!/bin/sh
# This is a comment!
roscore || true
./build-simplePaint-Desktop-Debug/simplePaint rrtstar
roslaunch rrtstar rrtStar.launch
exit 0