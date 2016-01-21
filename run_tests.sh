#!/bin/bash
# This script is designed for local usage.

# copy open-cv binary
./ubuntu_fix_cv2.sh

# start roscore to enable tests using ROS
echo "Starting roscore"
export ROS_MASTER_URI=http://localhost:11311
roscore &
echo "Waiting until roscore is started"
until rostopic list ; do sleep 1; done

echo "Starting gazebo"
rosrun gazebo_ros gzserver &
sleep 5

# Code coverage does not seem work without /nfs4 or /gpfs access
make test-nocover
RET=$?

# Cleanup after ourselves
pgrep -f ros | xargs kill -9
killall gzserver


if [ $RET == 0 ]; then
    echo -e "\033[32mTest sucessfull.\e[0m"
else
    echo -e "\033[31mTest failed.\e[0m See errors above."
fi

exit $RET
