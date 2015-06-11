#!/bin/bash

# This script lets you run all tests of this python pip package on a remote VM.
# 
# By running the following two commands from your local machine you can check 
# your code before uploading it to gerrit.
#
# Whenever you change the code and want to test (the whole set of tests) on a 
# deployed VM, you just have to use the following commands: 
#
# export DEV_VM=bbpce013.epfl.ch # ADAPT TO YOUR VM'S NAME
# ssh -t root@$DEV_VM 'ps aux | grep gzserver | grep -v grep || nohup /etc/init.d/gzserver restart 2>&1 >/dev/null' # start gzserver (needed by CLE tests)
# if [ ! -d ContinuousIntegration ]; then make; fi
# rsync -avzP -e ssh --exclude 'platform_venv' --delete . root@$DEV_VM:~/CLE
# ssh -t root@$DEV_VM 'cd ~/CLE && ./dev_run_all_tests_remote.sh'
#
# Note that this creates the directory if it is not there but synchronizes the 
# local and remote directories otherwise.
# The FIRST run may be very long since scipy will be built, but only once.
# Note further that by defining aliases and setting an env variable you can 
# boil down the complete test run to only one single command. 
# 
# (This script is part of the git repository because this way it is placed where
# it is actually needed. If we would put it in the wiki the risk is high that 
# a synchronisation between changes in the code and in this script are not down.)
#  
# author: deser@in.tum.de, axel.vonarnim@fortiss.org

NFS4_SW=/nfs4/bbp.epfl.ch/sw

function set_environment_and_load_modules() {
  source /opt/rh/python27/enable
  
  export MODULEPATH=$MODULEPATH:$NFS4_SW/neurorobotics/modulefiles
  export MODULEPATH=$MODULEPATH:$NFS4_SW/module/modulefiles
  module load boost/1.55zlib-rhel6-x86_64-gcc4.4
  
  module load ros/hydro-rhel6-x86_64-gcc4.4
  source $ROS_SETUP_FILE
  
  module load ros-hbp-packages/hydro-rhel6-x86_64-gcc4.4
  source $ROS_HBP_PACKAGES_SETUP_FILE
  
  export PYTHONPATH=$NFS4_SW/neurorobotics/ros-hbp-packages/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib/python2.7/site-packages:$PYTHONPATH
  
  # OpenCV
  module load opencv/2.4.9-rhel6-x86_64-gcc4.8.2
  # Gazebo
  module load gazebo/4.0-rhel6-x86_64-gcc4.8.2
  export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$NFS4_SW/neurorobotics/sdf/2.0/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib64/pkgconfig
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$NFS4_SW/neurorobotics/sdf/2.0/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib64
  export SDFormat_DIR=$NFS4_SW/neurorobotics/sdf/2.0/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib64/cmake/sdformat
  export ESVRENDER_MATERIAL_PATH=$NFS4_SW/neurorobotics/gazebo/4.0/rhel-6.5-x86_64/gcc-4.8.2/x86_64/share/gazebo-4.0
  # Ogre (needed by Gazebo)
  module load ogre/1.8.1-rhel6-x86_64-gcc4.8.2
  # TBB
  module load tbb/4.0.5-rhel6-x86_64-gcc4.4
  # Additional libs
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$NFS4_SW/neurorobotics/staging/lib64
  
  module load nest/2.2.2
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib64/compat-openmpi/lib
  export NRP_MODELS_DIRECTORY=/opt/hbp/gazebo/models
}

function run_all_tests() {
  # scl is the Software Collection tool used by Red Hat
  # Here we run the command "make test" with python 2.7
  scl enable python27 "make test"
}

set_environment_and_load_modules
run_all_tests
