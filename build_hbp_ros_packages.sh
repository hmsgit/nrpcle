#!/bin/bash -e
# ------------------------------------------------------------------
# [peppicel] Get all the customized or proper HBP ROS packages and build 
#            a GNU module out of them.
#            This script needs to be run on an HBP virtual machine 
#            that can access the NFS4. For example a CLEserver instance
#            is a good place. Before running that script, 
#            run "chmod a+w /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages"
#	     on a machine where you are logged with your username and where
#            the nfs4 is mounted.
# 	     After the script execution, run the opposite:
#            "chmod a-w /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages"
#
#            Some unusual yum packages are needed to build this:
#            tinyxml-devel, freeimage-devel, cmake, log4cxx-devel, libbuid-devel, poco-devel, yaml-cpp-devel
# -----------------------------------------------------------------------------------------------------------


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
module load ogre/1.8.1-rhel6-x86_64-gcc4.8.2
module load ros-thirdparty/hydro-rhel6-x86_64-gcc4.4

source $ROS_SETUP_FILE
source $ROS_THIRDPARTY_PACKAGES_SETUP_FILE

# Create a python venv in order to get the bases ROS install tools
virtualenv build_venv
. build_venv/bin/activate
pip install catkin_pkg
pip install empy
pip install PyYAML
pip install rospkg
pip install netifaces
pip install rosinstall
pip install rosinstall_generator

rm -rf build
mkdir -p build/src
cd build
cp -R ../GazeboRosPackage/* .

cd src
catkin_init_workspace

cd ..

# ATTENTION: There is problem with cmake: After installing the thirdparty packages fixed paths are written to the file /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-thirdparty/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/share/joint_limits_interface/cmake/joint_limits_interfaceConfig.cmake
# TODO: Manually remove '<custom_build_folder>/CLE/build/src/ros_control/hardware_interface/include' from line 96: 'set(_include_dirs "include; ...'
BOOST_ROOT=/nfs4/bbp.epfl.ch/sw/neurorobotics/boost/1.55-zlib/rhel-6.5-x86_64/gcc-4.4.7/x86_64/ catkin_make -DBoost_INCLUDE_DIR=$BOOST_INCLUDEDIR -DBoost_LIBRARY_DIRS=$BOOST_LIBDIR -DBoost_NO_BOOST_CMAKE=true -DTBB_INCLUDE_DIR=$TBB_INCLUDE_DIR

catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_INSTALL_PREFIX=/nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/ install

