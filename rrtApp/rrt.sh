#!/bin/sh
# This is a comment!
roscore || true
./build-simplePaint-Desktop-Debug/simplePaint rrt
roslaunch rrt rrt.launch
exit 0