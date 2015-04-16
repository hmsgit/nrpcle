#!/bin/bash
# This script is intended for external usage such as Jenkins builds,
# On a dev machine, the module test can be run only using line 33
# provided you run a roscore and gzserver

# Check out NRP platform
git clone ssh://bbpcode.epfl.ch/neurorobotics/CLE
git clone ssh://bbpcode.epfl.ch/neurorobotics/Models

export PYTHONPATH=$PYTHONPATH:$PWD/CLE/GazeboRosPackage/devel/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$PWD/CLE/hbp_nrp_cle
export NRP_MODELS_DIRECTORY=$PWD/Models

# Installing the ros package to control gazebo
cd CLE/GazeboRosPackage/src/
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
cd ../..

# start roscore and gzserver
echo "Starting roscore"
roscore &
echo "Waiting until roscore is started"
until rostopic list ; do sleep 1; done

echo "Starting gazebo"
rosrun gazebo_ros gzserver &
sleep 5

# Starting Module test
python CLE/hbp_nrp_cle/hbp_nrp_cle/tests/module/module_test.py

# Cleanup after ourselves
rm -rf CLE
rm -rf Models
pgrep -f ros | xargs kill -9
killall gzserver

