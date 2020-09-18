#!/bin/sh
# This is a comment!
roscore || true
./simplePaint rrt
roslaunch rrt rrt.launch
exit 0