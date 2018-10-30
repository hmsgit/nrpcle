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
This module tests the backend implementation of the simulation lifecycle
"""

import unittest
from mock import Mock, patch
from hbp_nrp_cle.robotsim.RobotManager import Robot, RobotManager

__author__ = 'Hossain Mahmud'

class TestRobotManager(unittest.TestCase):

    def setUp(self):
        self.mocked_gzhelper = patch("hbp_nrp_cle.robotsim.RobotManager.GazeboHelper").start()
        self.mocked_gzhelper.return_value.load_gazebo_model_file.return_value = ""

        self.robotManager = RobotManager()

    def tearDown(self):
        self.mocked_gzhelper.stop()

    def test_robot_manager_created_correctly(self):
        self.assertTrue('robots' in dir(self.robotManager), "Could not find attribute robots")
        self.assertTrue('_RobotManager__sceneHandler' in dir(self.robotManager), "Could not find attribute __sceneHandler")
        self.assertTrue('_RobotManager__retina_config' in dir(self.robotManager), "Could not find attribute __retina_config")

    def test_get_robot_dict(self):
        self.assertFalse(self.robotManager.get_robot_dict(), "Newly created robot manager is not empty")
        self.robotManager.robots = {'key': 'value'}
        self.assertTrue('key' in self.robotManager.get_robot_dict())
        self.assertTrue(self.robotManager.get_robot_dict()['key'] == 'value')

    def test_get_robot_dict_clone(self):
        self.assertFalse(self.robotManager.get_robot_dict(), "Newly created robot manager is not empty")
        self.robotManager.robots = {'key': 'value'}
        robots = self.robotManager.get_robot_dict_clone()
        # robots should not be the same object, but should have same values
        self.assertIsNot(robots, self.robotManager.robots)
        self.assertTrue(robots == self.robotManager.robots)

    def test_get_robot(self):
        self.robotManager.robots = {'key': 'value'}
        robot = self.robotManager.get_robot('key')
        self.assertTrue(robot == 'value')

    def test_add_robot(self):
        robot = Robot('id',  '/some/sdf/path', 'some name', None, False)
        self.robotManager.add_robot(robot)
        self.assertTrue(robot is self.robotManager.robots[robot.id])
        self.assertRaises(Exception, self.robotManager.add_robot, robot)

    def test_remove_robot(self):
        self.robotManager.robots = {'key': 'value'}
        self.robotManager.remove_robot('key')
        self.assertFalse(self.robotManager.robots)

    def test_remove_all_robots(self):
        self.robotManager.robots = {'key': 'value', 'key2': 'another value'}
        self.robotManager.remove_all_robots()
        self.assertFalse(self.robotManager.robots)

    def test_init_scene_handler(self):
        gz = self.robotManager.init_scene_handler()
        self.assertTrue(gz is self.mocked_gzhelper.return_value)

    def test_load_robot_in_scene(self):
        self.robotManager._RobotManager__sceneHandler = self.mocked_gzhelper()

        self.robotManager.load_robot_in_scene('myrobot')

        mockedRobot = Mock()
        mockedRobot.return_value = {'id': 'id', 'SDFFileAbsPath': 'sdf', 'pose': 'pose'}

        self.robotManager.robots = {'myrobot': mockedRobot}

        self.robotManager.load_robot_in_scene('myrobot', 'retina')
        self.robotManager._RobotManager__sceneHandler\
            .load_gazebo_model_file.assert_called_once_with(str(mockedRobot.id), mockedRobot.SDFFileAbsPath, mockedRobot.pose, 'retina')

    def test_scene_handler(self):
        self.assertRaises(Exception, self.robotManager.scene_handler)
        sceneHandler = "/some/scene/handler"
        self.robotManager._RobotManager__sceneHandler = sceneHandler
        self.assertTrue(self.robotManager.scene_handler() is sceneHandler)

    def test_convertXSDPosetoPyPose(self):
        pose = None
        self.assertTrue(self.robotManager.convertXSDPosetoPyPose(pose) is None)

        pose = Mock()
        self.mocked_pose = patch("hbp_nrp_cle.robotsim.RobotManager.Pose").start()
        self.mocked_pose.return_value.postion = "mocked_pose_position"
        self.mocked_transformation = patch("hbp_nrp_cle.robotsim.RobotManager.transformations").start()

        pose.ux.return_value = 1
        self.assertTrue(self.robotManager.convertXSDPosetoPyPose(pose), self.mocked_pose.return_value)

        pose.ux = None
        self.assertTrue(self.robotManager.convertXSDPosetoPyPose(pose), self.mocked_pose.return_value)

        self.mocked_transformation.quaternion_from_euler.return_value = [None, None, None, None]
        self.assertTrue(self.robotManager.convertXSDPosetoPyPose(pose), self.mocked_pose.return_value)

        self.mocked_pose.stop()
        self.mocked_transformation.stop()
