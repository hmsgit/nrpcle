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
CLE unit test
"""

from hbp_nrp_cle.cle.ClosedLoopEngine import ClosedLoopEngine
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainControlAdapter, MockBrainCommunicationAdapter
from hbp_nrp_cle.mocks.tf_framework import MockTransferFunctionManager
from geometry_msgs.msg import Point, Pose, Quaternion

import unittest
import threading
from mock import Mock, patch, MagicMock

__author__ = 'Nino Cauli'



# all the methods are inherited from unittest.TestCase
class TestClosedLoopEngine(unittest.TestCase):

    def setUp(self):
        # Sets up the cle and the mocks for the adapters.

        rca = MockRobotControlAdapter()
        rcm = MockRobotCommunicationAdapter()
        self.__bca = MockBrainControlAdapter()
        self.__bca.load_network_from_file = MagicMock()
        bcm = MockBrainCommunicationAdapter()
        self.__tfm = MockTransferFunctionManager()
        self.__tfm.hard_reset_brain_devices = MagicMock()

        # These patches are to avoid timeouts during the GazeboHelper instantiations in the ClosedLoopEngine.
        # They won't be necessary as soon as the ClosedLoopEngine won't embed a GazeboHelper anymore
        # (see related comments there)
        patch('hbp_nrp_cle.robotsim.GazeboHelper.rospy.wait_for_service').start()
        patch('hbp_nrp_cle.robotsim.GazeboHelper.rospy.ServiceProxy').start()

        self.__cle = ClosedLoopEngine(rca, rcm, self.__bca, bcm, self.__tfm, 0.01)

    def test_run_step(self):
        self.__cle.initialize("foo")
        self.assertEqual(self.__cle.run_step(0.01), 0.01)

    def test_get_time(self):
        self.__cle.initialize("foo")
        self.__cle.run_step(0.05)
        self.__cle.wait_step()
        self.assertEqual(self.__cle.simulation_time, 0.05)

    def test_start_stop(self):
        self.__cle.initialize("foo")

        def stopcle(*args, **kwargs):
            args[0].stop()

        t = threading.Timer(2.0, stopcle, [self.__cle])
        t.start()
        self.__cle.start()
        self.assertGreater(self.__cle.real_time, 0.0)

    def test_reset(self):
        self.__cle.initialize("foo")
        self.__cle.run_step(0.05)
        self.__cle.wait_step()
        self.__cle.reset()
        self.assertEqual(self.__cle.simulation_time, 0.0)
        self.assertEqual(self.__cle.real_time, 0.0)

    def test_reset_world_backwards_compatibility(self):
        self.__cle.rca = Mock()
        self.__cle.rca.reset_world = Mock()

        self.__cle.reset_world()
        self.__cle.rca.reset_world.assert_called_with(self.__cle.initial_models, self.__cle.initial_lights)

    def test_reset_world(self):

        mock_sdf = "<sdf></sdf>"

        mock_models_dict = {'box_0': mock_sdf}
        mock_lights_dict = {'light1': mock_sdf}

        self.__cle.gazebo_helper.parse_world_file = Mock(return_value=(mock_models_dict, mock_lights_dict))

        self.__cle.rca = Mock()

        mock_sdf_world = mock_sdf
        self.__cle.reset_world(mock_sdf_world)

        self.__cle.rca.reset_world.assert_called_with(mock_models_dict, mock_lights_dict)

    def test_reset_brain(self):
        self.__cle.load_network_from_file = Mock()
        self.__cle._ClosedLoopEngine__network_configuration = {}
        self.__cle.reset_brain()
        self.assertEquals(1, self.__cle.load_network_from_file.call_count)

    def test_shutdown(self):
        self.__cle.initialize("foo")
        self.__cle.shutdown()

    def test_reset_robot_pose(self):
        self.__cle.gazebo_helper.set_model_pose = Mock()
        pose = Pose()
        pose.position = Point(0, 0, 0)
        pose.orientation = Quaternion(0, 0, 0, 1)

        self.__cle.initial_robot_pose = pose
        self.__cle.reset_robot_pose()
        self.__cle.gazebo_helper.set_model_pose.assert_called_with('robot', pose)

    def load_network_from_file(self):

        populations = {
            'index1': 1,
            'list1': [1, 2, 3],
            'slice1': {'from': 1, 'to': 2, 'step': 1}

        }

        # load brain should not work when CLE is not initialized
        self.__bca.load_network_from_file.assert_called_with("brain.py", populations)
        self.assertEqual(self.__bca.load_network_from_file.call_count, 0)

        # Happy case
        self.__cle.initialize("foo")
        self.__cle.start()
        self.assertEqual(self.__cle.running, True)
        self.__cle.load_network_from_file("brain.py", populations)
        self.assertEqual(self.__cle.running, False)
        self.__bca.load_network_from_file.assert_called_with("brain.py", populations)
        self.assertEqual(self.bca.is_alive(), False)
        self.__tfm.hard_reset_brain_devices.assert_called()


if __name__ == '__main__':
    unittest.main()
