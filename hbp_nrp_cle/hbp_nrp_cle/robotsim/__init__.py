"""
The robotsim package deals with the world simulation. The package provides the abstract interfaces
as well an implementation using ROS and Gazebo
"""

__author__ = 'GeorgHinkel'

from .RobotInterface import IRobotCommunicationAdapter, IRobotControlAdapter

ROS_S_SPAWN_SDF_LIGHT = '/gazebo/spawn_sdf_light'
ROS_S_SPAWN_SDF_MODEL = '/gazebo/spawn_sdf_model'
