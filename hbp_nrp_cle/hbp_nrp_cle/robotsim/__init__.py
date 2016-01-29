"""
The robotsim package deals with the world simulation. The package provides the abstract interfaces
as well an implementation using ROS and Gazebo
"""

__author__ = 'GeorgHinkel'

from .RobotInterface import IRobotCommunicationAdapter, IRobotControlAdapter

GZROS_S_SPAWN_SDF_LIGHT = '/gazebo/spawn_sdf_light'
GZROS_S_SPAWN_SDF_MODEL = '/gazebo/spawn_sdf_model'
GZROS_S_GET_WORLD_PROPERTIES = '/gazebo/get_world_properties'
GZROS_S_SET_MODEL_STATE = '/gazebo/set_model_state'
GZROS_S_DELETE_MODEL = '/gazebo/delete_model'
GZROS_S_DELETE_LIGHT = '/gazebo/delete_light'
GZROS_S_DELETE_LIGHTS = '/gazebo/delete_lights'
GZROS_S_GET_LIGHTS_NAME = '/gazebo/get_lights_name'
