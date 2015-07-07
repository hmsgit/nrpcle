#!/bin/bash

# start roscore to enable tests using ROS
echo "Starting roscore"
roscore &
echo "Waiting until roscore is started"
until rostopic list ; do sleep 1; done

echo "Starting gazebo"
rosrun gazebo_ros gzserver &
sleep 5

make test

# Cleanup after ourselves
pgrep -f ros | xargs kill -9
killall gzserver

