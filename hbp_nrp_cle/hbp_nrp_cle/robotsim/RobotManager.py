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

from geometry_msgs.msg import Pose
import tf.transformations as transformations

from .GazeboHelper import GazeboHelper
from hbp_nrp_cleserver.server.ROSLaunch import ROSLaunch

logger = logging.getLogger(__name__)

__author__ = 'Hossain Mahmud'


class Robot(object):
    """
    Robot model structure to be used in the CLE
    """
    def __init__(self, rid, sdf_abs_path, display_name, pose,
                 is_custom=False, roslaunch_abs_path=None):
        self.id = rid
        self.SDFFileAbsPath = sdf_abs_path
        self.displayName = display_name
        self.pose = pose    # quaternion pose: geometry_msgs.msg.Pose
        self.isCustom = is_custom
        self.rosLaunchAbsPath = roslaunch_abs_path

        self.rosLaunch = None


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

    def get_robot_dict(self):
        """
        Gets a reference to the robot dictionary with robot id as key and Robot object as value
        :return: robot list
        """
        return self.robots

    def get_robot_dict_clone(self):
        """
        :return: A copy of the robot dictionary with robot id as key and Robot object as value
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

        if robot.rosLaunchAbsPath:
            robot.ROSLaunch = ROSLaunch(robot.rosLaunchAbsPath)

        try:
            logger.info("Adding robot {id} from {path}: "
                        .format(id=robot.id, path=robot.SDFFileAbsPath))
            self.scene_handler().load_gazebo_model_file(
                str(robot.id), robot.SDFFileAbsPath, robot.pose, self.retina_config)
        except Exception as e:
            del self.robots[robot.id]
            raise Exception("Error loading {robot} into the scene. {err}"
                            .format(robot=robot.id, err=str(e)))

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

    def scene_handler(self):
        """
        Helper function to ensure scene handler is not being accessed before initializing gazebo
        :return: A scene handler object capable of manipulating the 3D scene, e.g., gazebo scene
        :raises Exception is raised if accessed before initializing the physics engine
        """
        if not self.__sceneHandler:
            raise Exception("Trying to access scene handler without initializing physics engine")
        return self.__sceneHandler

    @property
    def retina_config(self):
        """
        Gets retina config
        """
        return self.__retina_config

    @retina_config.setter
    def retina_config(self, config):
        """
        Sets retina config

        :param config: retina config to use
        """
        self.__retina_config = config

    def delete_robot_from_scene(self, robot_id):
        """
        Deletes a robot from the 3D scene

        :param robot_id: Id of the robot to be deleted. Ignores the call if id is missing
        :return: -
        """
        robot = self.robots.get(robot_id, None)
        if robot:
            self.scene_handler().delete_model_proxy(str(robot.id))
        else:
            logger.info("Cannot delete from RobotManager: id {} does not exist".format(robot_id))

    def shutdown(self):
        """
        Cleans up any process or resource used by the manager
        """
        for robot in self.robots.itervalues():
            if robot.rosLaunch:
                robot.rosLaunch.shutdown()

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

        # REST request has no ux defined. Exc DOM however has one
        if getattr(xsd_pose, 'ux', None) is not None:
            rpose.orientation.x = xsd_pose.ux
            rpose.orientation.y = xsd_pose.uy
            rpose.orientation.z = xsd_pose.uz
            rpose.orientation.w = xsd_pose.theta
        else:
            roll = xsd_pose.roll if xsd_pose.roll is not None else 0
            pitch = xsd_pose.pitch if xsd_pose.pitch is not None else 0
            yaw = xsd_pose.yaw if xsd_pose.yaw is not None else 0

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
