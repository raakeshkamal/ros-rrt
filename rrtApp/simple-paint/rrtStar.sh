#!/bin/sh
# This is a comment!
roscore || true
./simplePaint rrtstar
roslaunch rrtstar rrtStar.launch
exit 0