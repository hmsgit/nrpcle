# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
#!/usr/bin/env python
"""Implementation of the robot control adapter using ros and gazebo"""

from hbp_nrp_cle.robotsim.AsynchronousServiceProxy import AsynchonousRospyServiceProxy
from hbp_nrp_cle.robotsim.GazeboHelper import GazeboHelper
from hbp_nrp_cle.robotsim.RobotInterface import IRobotControlAdapter
import rospy
import math
# pylint: disable=E0611
from gazebo_msgs.srv import GetPhysicsProperties, GetWorldProperties, \
    SetPhysicsProperties, AdvanceSimulation
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion
import logging


logger = logging.getLogger(__name__)
# Info messages sent to this logger will be forwarded as notifications
user_notifications_logger = logging.getLogger('hbp_nrp_cle.user_notifications')

__author__ = 'NinoCauli'


class RosControlAdapter(IRobotControlAdapter):
    """
    Represents a robot simulation adapter actually using ROS
    """

    def __init__(self):
        self.__persistent_services = []
        rospy.wait_for_service('/gazebo/get_physics_properties')
        self.__get_physics_properties = rospy.ServiceProxy(
            '/gazebo/get_physics_properties', GetPhysicsProperties, persistent=True)
        self.__persistent_services.append(self.__get_physics_properties)

        rospy.wait_for_service('/gazebo/get_world_properties')
        self.__get_world_properties = rospy.ServiceProxy(
            '/gazebo/get_world_properties', GetWorldProperties, persistent=True)
        self.__persistent_services.append(self.__get_world_properties)

        rospy.wait_for_service('/gazebo/set_physics_properties')
        self.__set_physics_properties = rospy.ServiceProxy(
            '/gazebo/set_physics_properties', SetPhysicsProperties, persistent=True)
        self.__persistent_services.append(self.__set_physics_properties)

        rospy.wait_for_service('/gazebo/pause_physics')
        self.__pause_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty, persistent=True)
        self.__persistent_services.append(self.__pause_client)

        rospy.wait_for_service('/gazebo/reset_sim')
        self.__reset = rospy.ServiceProxy('/gazebo/reset_sim', Empty, persistent=True)
        self.__persistent_services.append(self.__reset)

        rospy.wait_for_service('/gazebo/end_world')
        self.__endWorld = rospy.ServiceProxy('/gazebo/end_world', Empty, persistent=True)
        self.__persistent_services.append(self.__endWorld)

        rospy.wait_for_service('/gazebo/advance_simulation')
        self.__advance_simulation = AsynchonousRospyServiceProxy(
            '/gazebo/advance_simulation', AdvanceSimulation, persistent=True)
        self.__persistent_services.append(self.__advance_simulation)

        self.__time_step = 0.0
        self.__is_initialized = False

        self.gazebo_helper = GazeboHelper()

    def initialize(self):
        """
        Initializes the world simulation control adapter
        """
        if not self.__is_initialized:
            physics = self.__get_physics_properties()
            paused = physics.pause
            if not paused:
                self.__pause_client()
            self.__time_step = physics.time_step
            self.__is_initialized = True

        logger.info("Robot control adapter initialized")
        return self.__is_initialized

    @property
    def time_step(self):
        """
        Gets the physics simulation time step in seconds

        :param dt: The physics simulation time step in seconds
        :return: The physics simulation time step in seconds
        """
        return self.__time_step

    def set_time_step(self, time_step):
        """
        Sets the physics simulation time step in seconds

        :param time_step: The physics simulation time step in seconds
        :return: True, if the physics simulation time step is updated, otherwise False
        """
        physics = self.__get_physics_properties()
        success = self.__set_physics_properties(
            time_step,
            physics.max_update_rate,
            physics.gravity,
            physics.ode_config)
        if success:
            self.__time_step = time_step
            logger.info("new time step = %f", self.__time_step)
        else:
            logger.warn("impossible to set the new time step")
        return success

    @property
    def is_paused(self):
        """
        Queries the current status of the physics simulation

        :return: True, if the physics simulation is paused, otherwise False
        """
        physics = self.__get_physics_properties()
        paused = physics.pause
        return paused

    @property
    def is_alive(self):
        """
        Queries the current status of the world simulation

        :return: True, if the world simulation is alive, otherwise False
        """
        logger.debug("Getting the world properties to check if we are alive")
        world = self.__get_world_properties()
        success = world.success
        return success

    def run_step_async(self, dt):
        """
        Runs the world simulation for the given CLE time step in seconds

        :param dt: The CLE time step in seconds
        """

        # implement modulo to avoid floating point precision quirks in math.fmod or from direct %
        # without this explicit fix, values such as 0.999 % 0.001 != 0.0 due to float accuracy
        # https://en.wikipedia.org/wiki/Modulo_operation : a - (n * int(a/n))
        r = dt - (self.__time_step * math.floor(dt / self.__time_step))
        if r == 0.0:
            steps = dt / self.__time_step
            logger.debug("Advancing simulation")

            return self.__advance_simulation(steps)

        else:
            logger.error("dt is not multiple of the physics time step")
            raise ValueError("dt is not multiple of the physics time step")

    def run_step(self, dt):
        """
        Runs the world simulation for the given CLE time step in seconds

        :param dt: The CLE time step in seconds
        """

        return self.run_step_async(dt).result()

    def shutdown(self):
        """
        Shuts down the world simulation
        """
        logger.info("Shutting down the world simulation")

        for service in self.__persistent_services:
            service.close()

        logger.info("Robot control adapter stopped")

        # Do not call endWorld here, it makes Gazebo Stop !

    def reset(self):
        """
        Resets the physics simulation
        """
        logger.info("Resetting the world simulation")
        self.__reset()

    def reset_world(self, models, lights):
        """
        Resets the world (robot excluded) to the state described by
        models and lights

        :param models: A dictionary containing pairs
            (model_name: {'model_sdf': sdf, 'model_state_sdf': sdf}).
        :param lights: A dictionary containing pairs (light_name: light sdf).
        """
        logger.debug("Resetting the Environment")

        # MODELS
        # get the list of models' name currently the sim from gazebo
        world_properties = self.__get_world_properties()

        # robot model doesn't belong to the environment, so discard them from the active set
        # we assume that the robot name contains the 'robot' substring
        active_model_set = \
            {m_name for m_name in world_properties.model_names if 'robot' not in m_name}
        original_model_set = frozenset(models.keys())

        logger.debug("active_model_set: %s", active_model_set)
        logger.debug("original_model_set: %s", original_model_set)

        # LIGHTS
        # get from gazebo the name of the lights in the scene
        world_lights = self.gazebo_helper.get_lights_name_proxy()

        # filter out sun from the world lights
        active_lights_set = \
            {l_name for l_name in world_lights.light_names if 'sun' not in l_name}
        original_lights_set = frozenset(lights.keys())

        logger.debug("active_lights_set: %s", active_lights_set)
        logger.debug("original_lights_set: %s", original_lights_set)

        # delete LIGHTS
        for light_name in active_lights_set:
            user_notifications_logger.info("Deleting: %s", light_name)
            self.gazebo_helper.delete_light_proxy(light_name)

        # delete MODELS
        for model_name in active_model_set:
            user_notifications_logger.info("Deleting: %s", model_name)
            self.gazebo_helper.delete_model_proxy(model_name)

        initial_pose = Pose()
        initial_pose.position = Point(0, 0, 0)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)

        # respawn LIGHTS
        for light_name in original_lights_set:
            user_notifications_logger.info("Loading: %s", light_name)
            self.gazebo_helper.spawn_entity_proxy(light_name, lights[light_name],
                                                  "", initial_pose, "")

        # respawn MODELS
        for model_name in original_model_set:
            user_notifications_logger.info("Loading: %s", model_name)

            self.gazebo_helper.spawn_entity_proxy(model_name, models[model_name]['model_sdf'],
                                                  "", initial_pose, "")
