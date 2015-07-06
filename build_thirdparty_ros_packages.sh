#!/bin/bash -e
# ------------------------------------------------------------------
# [peppicel] Get all the needed third party ROS packages and build 
#            a GNU module out of them.
#            This script needs to be run on an HBP virtual machine 
#            that can access the NFS4. For example a CLEserver instance
#            is a good place. Before running that script, 
#            run "chmod a+w /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-thirdparty"
#	     on a machine where you are logged with your username and where
#            the nfs4 is mounted.
# 	     After the script execution, run the opposite:
#            "chmod a-w /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-thirdparty"
# ------------------------------------------------------------------

# We need python 2.7
source /opt/rh/python27/enable

# Get the proper modules
export MODULEPATH=$MODULEPATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/modulefiles
module load boost/1.55zlib-rhel6-x86_64-gcc4.4
module load ros/hydro-rhel6-x86_64-gcc4.4
module load gazebo/4.0-rhel6-x86_64-gcc4.8.2
module load opencv/2.4.9-rhel6-x86_64-gcc4.8.2
module load sdf/2.0-rhel6-x86_64-gcc4.4
module load tbb/4.0.5-rhel6-x86_64-gcc4.4

source $ROS_SETUP_FILE

# Create a python venv in order to get the bases ROS install tools
virtualenv build_venv
. build_venv/bin/activate

USE_DEV_PI="-i http://bbpgb019.epfl.ch:9090/simple"

pip install $USE_DEV_PI catkin_pkg
pip install $USE_DEV_PI empy
pip install $USE_DEV_PI PyYAML
pip install $USE_DEV_PI rospkg
pip install $USE_DEV_PI netifaces
pip install $USE_DEV_PI rosinstall
pip install $USE_DEV_PI rosinstall_generator

#pip install rosdep 
#sudo pip install --upgrade setuptools # Was necessary in the virtual machine

rm -rf build
mkdir -p build/src
cd build/src
wstool init

# ROS Control
wstool merge https://raw.github.com/ros-controls/ros_control/indigo-devel/ros_control.rosinstall

# ROS Bridge
roslocate info rosbridge_suite | wstool merge -

# ROS auth (dependency of ROS bridge)
roslocate info rosauth | wstool merge -

# ROS image_common (needed by gazebo_ros_control)
roslocate info image_common | wstool merge -

wstool update
cd ..
rm -rf src/ros_control/rqt_controller_manager

catkin_make   -DCATKIN_ENABLE_TESTING=0 -DCMAKE_INSTALL_PREFIX=/nfs4/bbp.epfl.ch/sw/neurorobotics/ros-thirdparty/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/ install
