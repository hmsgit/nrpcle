# gazebo_ros_pkgs

Wrappers, tools and additional API's for using ROS with the Gazebo simulator, patched for HBP. This packages replaces the one found in the official repositories.

### Installation
Copy this folder inside the src folder of a catkin workspace and then build with catkin_make

### Usage
source CATKIN_WS/devel/setup.bash
rosrun gazebo_ros gazebo

### tinyXML problem
if you have problems with the tinyXML compiling the gazebo_ros_pkgs, clone the missing package (tinyXML) into my catkin_workspace/src folder like

git clone https://github.com/ros/cmake_modules.git