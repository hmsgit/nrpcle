#!/bin/bash

# Installing the ros package to control gazebo
cd GazeboRosPackage/src/
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
# catkin_make install
cd ..

# start roscore to enable tests using ROS
echo "Starting roscore"
roscore &
echo "Waiting until roscore is started"
until rostopic list ; do sleep 1; done

echo "Starting gazebo"
rosrun gazebo_ros gzserver &
sleep 5

export IGNORE_LINT="platform_venv|GazeboRosPackage/devel/lib|GazeboRosPackage/build|GazeboRosPackage/devel|GazeboRosPackage/install|GazeboRosPackage/src/gazebo_ros_pkgs|build|hbp_nrp_cle/hbp_nrp_cle/bibi_config/generated|demo/"

make verify

# Cleanup after ourselves
pgrep -f ros | xargs kill -9
killall gzserver

