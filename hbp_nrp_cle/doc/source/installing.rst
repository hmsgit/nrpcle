Installing the CLE
==================

The CLE is not intended to be installed separately. Instead, it is recommended to use the CLE through the NRP platform that instantiates the CLE according to a good configuration and manages a simulation.

However, if you for whatever reason want to install the CLE isolated from the NRP platform, you can do so by following the instructions of this section.

Install prerequisites
---------------------

.. code-block:: bash

   sudo apt-get install python-dev python-h5py libxslt-1dev python-lxml autogen automake libtool build-essential autoconf libltdl7-dev libreadline6-dev libncurses5-dev libgsl0-dev python-all-dev python-numpy python-scipy python-matplotlib ipython python-pynn python-pip


Installation of  ROS
--------------------
This assumes work on Ubuntu (>= 13.10), otherwise use `ROS Hydro <http://wiki.ros.org/hydro/Installation/Ubuntu>`_ instead

* Add the Ubuntu package sources and `install ROS <http://wiki.ros.org/indigo/Installation/Ubuntu>`_. In short you have to run:

  .. code-block:: bash
            
          sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
          wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
          sudo apt-get update
          sudo apt-get install ros-indigo-desktop-full


* To add the `Gazebo <http://gazebosim.org/tutorials?tut=install_ubuntu&cat=installation>`_. Ubuntu package sources, run the following commands:

  .. code-block:: bash

      sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
      wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
      sudo apt-get update
      sudo apt-get install gazebo4
      sudo apt-get install ros-indigo-gazebo4-msgs ros-indigo-gazebo4-plugins ros-indigo-gazebo4-ros ros-indigo-gazebo4-ros-control ros-indigo-gazebo4-ros-pkgs ros-indigo-joint-limits-interface


Installation of the NEST simulator
----------------------------------

1. Download NEST 2.2.2 `(link) <http://www.nest-simulator.org/download/gplreleases/nest-2.2.2.tar.gz>`_.

   .. code-block:: bash

      wget http://www.nest-simulator.org/download/gplreleases/nest-2.2.2.tar.gz
      tar zxvf nest-2.2.2.tar.gz
      cd nest-2.2.2


   .. note::
      It is important to not download any newer version, particularly not 2.4.2

2. Build the sources and install the software:

   .. code-block:: bash

       ./configure --prefix=/opt/nest
       make
       sudo make install

.. _acquisition:

Acquire the sources of the CLE
------------------------------

Get the code for the CLE repository. Create a folder to where you keep your source (e.g.,
``projects/HBP``)

.. code-block:: bash

     mkdir <path-to-project>
     cd <path-to-project>
     git clone ssh://<user>@bbpcode.epfl.ch/neurorobotics/CLE


Building the patched Gazebo Plugin
----------------------------------

.. code-block:: bash

    source /opt/ros/indigo/setup.bash
    cd CLE/GazeboRosPackage
    catkin_make


Setup the Gazebo Client
-----------------------

If the Gazebo Client will be used for visualization, link the models to the ~/.gazebo/models folder:

.. code-block:: bash

    mkdir -p ~/.gazebo/models
    for a in ${GAZEBO_MODELS[@]}; do ln -s $NRP_MODELS_DIRECTORY/$a ~/.gazebo/models/; done

Install CLE dependencies
------------------------

.. code-block:: bash

    cd <root directory of CLE>
    make devinstall
