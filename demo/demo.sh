#!/bin/bash

# start roscore
roscore &
until rostopic list; do
    sleep 1
done

# start gazebo with environment
#rosrun gazebo_ros gazebo env.sdf

echo "Starting gazebo"
roslaunch gazebo_ros hbp_husky.launch &

sleep 10

# start CLE + NEST
echo "Running demo gazebo"
#python -m cProfile -o output.pstats -s cumtime demo.py
python demo.py

# clean up
killall gzclient
killall gzserver
killall roscore
