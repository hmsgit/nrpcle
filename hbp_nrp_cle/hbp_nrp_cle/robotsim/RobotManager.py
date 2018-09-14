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

"""
This module manages robot resources in CLE for a running simulation. Addition, deletion, update,
and en/disabling of robots should be handled in this module.

CLE Assembly (cleserver) creates an object of this class, and passes around.
"""

import copy
import logging
from .GazeboHelper import GazeboHelper

from geometry_msgs.msg import Pose
import tf.transformations as transformations

logger = logging.getLogger(__name__)

__author__ = 'Hossain Mahmud'


class Robot(object):
    """
    Robot model structure to be used in the CLE
    """
    def __init__(self, rid, sdf_rel_path, display_name, pose, is_custom=False):
        self.id = rid
        self.SDFFileRelPath = sdf_rel_path
        self.displayName = display_name
        self.pose = pose
        self.isCustom = is_custom


class RobotManager(object):
    """
    Manages robot resouces in the CLE
    """

    def __init__(self):
        """
        Create a robot manager
        """
        self.robots = {}
        self.__sceneHandler = None
        self.__retina_config = None

    def get_robot_list(self):
        """
        Gets a reference to the robot list
        :return: robot list
        """
        return self.robots

    def get_robot_list_clone(self):
        """
        :return: A copy of the robot list
        """
        return copy.deepcopy(self.robots)

    def get_robot(self, robot_id):
        """
        Get Robot instance of the robot of the given id

        :param robot_id: id of the robot
        :return: A cle.robotsim.Robot object
        """
        return self.robots[robot_id]

    def add_robot(self, robot):
        """
        Add a robot object

        :param robot: A cle.robotsim.Robot object
        """
        if robot.id in self.robots:
            raise Exception("Robot name already exists.")
        self.robots[robot.id] = robot

    def remove_robot(self, robot_id):
        """
        Remove robot of the given Id

        :param robot_id: id of the robot to be removed
        :return: -
        """
        del self.robots[robot_id]

    def remove_all_robots(self):
        """
        Remove all robots from the manager

        :return: -
        """
        # pylint: disable=unused-variable
        for k in self.robots.keys():
            del self.robots[k]

    def init_scene_handler(self):
        """
        Returns a reference to the active scene handler for the physics simulator.
        Can only be called after gazebo is started.

        :return: a reference to the active scene handler for the physics simulator
        """
        # TODO: GazeboHelper blocks thread if gazebo hasn't been initialized
        # FIXME: Implemnet busy-wait-timeout in GazeboHelper
        if not self.__sceneHandler:
            self.__sceneHandler = GazeboHelper()
        return self.__sceneHandler

    def load_robot_in_scene(self, robot_id, retina_config=None):
        """
        Loads a robot in the gazebo scene

        :param robot_id: Id of the robot to be loaded. Ignores the call if Id is missing
        :param retina_config: Any retina config. Saves it for the later use when spawning robots
        :return: -
        """
        if retina_config:
            self.__retina_config = retina_config

        robot = self.robots.get(robot_id, None)
        if robot:
            self.scene_handler().load_gazebo_model_file(
                str(robot.id), robot.SDFFileRelPath, robot.pose, self.__retina_config)
        else:
            logger.info("Robot ID does not exist.")

    def scene_handler(self):
        """
        Helper function to ensure scene handler is not being accessed before initializing gazebo
        :return: GazeboHelper
        """
        if not self.__sceneHandler:
            raise Exception("Trying to access scene handler without initializing physics engine")
        return self.__sceneHandler

    @staticmethod
    def convertXSDPosetoPyPose(xsd_pose):
        """
        Converts a xml DOM pose object to python Pose object

        :param xsd_pose: DOM object reference containing the pose
        :return: a converted Pose object
        """
        if xsd_pose is None:
            return None

        rpose = Pose()
        rpose.position.x = xsd_pose.x
        rpose.position.y = xsd_pose.y
        rpose.position.z = xsd_pose.z

        if xsd_pose.ux is not None:
            rpose.orientation.x = xsd_pose.ux
            rpose.orientation.y = xsd_pose.uy
            rpose.orientation.z = xsd_pose.uz
            rpose.orientation.w = xsd_pose.theta
        else:
            roll = 0
            pitch = 0
            yaw = 0

            if xsd_pose.roll is not None:
                roll = xsd_pose.roll
            if xsd_pose.pitch is not None:
                pitch = xsd_pose.pitch
            if xsd_pose.yaw is not None:
                yaw = xsd_pose.yaw

            quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)

            rpose.orientation.x = quaternion[0]
            rpose.orientation.y = quaternion[1]
            rpose.orientation.z = quaternion[2]
            rpose.orientation.w = quaternion[3]

        if rpose.orientation.x is None or rpose.orientation.y is None or \
                        rpose.orientation.z is None or rpose.orientation.w is None:
            rpose.orientation.x = 0.0
            rpose.orientation.y = 0.0
            rpose.orientation.z = 0.0
            rpose.orientation.w = 1.0

        return rpose
