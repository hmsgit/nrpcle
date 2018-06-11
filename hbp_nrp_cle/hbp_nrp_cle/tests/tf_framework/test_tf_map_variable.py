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
from hbp_nrp_cle.robotsim.RobotInterface import Topic
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.tf_framework import config
from hbp_nrp_cle.mocks.robotsim._MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, \
    MockPublishedTopic
from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import MockBrainCommunicationAdapter
from hbp_nrp_cle.tests.tf_framework.MockBrain import MockPopulation

from mock import Mock, patch
import unittest

__author__ = 'sebastiankrach'

MockOs = Mock()
MockOs.environ = {'NRP_SIMULATION_DIR': '/somewhere/near/the/rainbow'}
@patch("hbp_nrp_cle.common.os", new=MockOs)
class TestTransferFunctionVariables(unittest.TestCase):

    def setUp(self):
        nrp.start_new_tf_manager()
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

        brain.actors = MockPopulation(range(0, 60))
        brain.sensors = MockPopulation(range(45, 645))
        config.brain_root = brain

    def test_tf_initialize_local_mapping(self):
        @nrp.MapVariable("shared_var", initial_value=2)
        @nrp.Neuron2Robot(Topic("/vars/shared1", float))
        def echo_shared_var(t, shared_var):
            return shared_var.value

        @nrp.MapVariable("shared_var", initial_value=5)
        @nrp.Neuron2Robot(Topic("/vars/shared2", float))
        def echo_shared_var2(t, shared_var):
            return shared_var.value

        nrp.initialize("MyTransferFunctions")

        topic1 = echo_shared_var.topic
        topic2 = echo_shared_var2.topic

        config.active_node.run_neuron_to_robot(1)

        config.active_node.run_neuron_to_robot(2)

        self.assertEqual(len(topic1.sent), 2)
        self.assertEqual(len(topic2.sent), 2)

        self.assertListEqual(topic1.sent, [2, 2])
        self.assertListEqual(topic2.sent, [5, 5])

    def test_tf_change_locals(self):

        @nrp.MapVariable("shared_var", initial_value=2)
        @nrp.Neuron2Robot(Topic("/vars/shared1", float))
        def echo_shared_var(t, shared_var):
            shared_var.value = shared_var.value + t
            return shared_var.value

        @nrp.MapVariable("shared_var", initial_value=5)
        @nrp.Neuron2Robot(Topic("/vars/shared2", float))
        def echo_shared_var2(t, shared_var):
            shared_var.value = shared_var.value - t
            return shared_var.value

        nrp.initialize("MyTransferFunctions")

        topic1 = echo_shared_var.topic
        topic2 = echo_shared_var2.topic

        config.active_node.run_neuron_to_robot(1)
        config.active_node.run_robot_to_neuron(1)

        config.active_node.run_neuron_to_robot(2)
        config.active_node.run_robot_to_neuron(2)

        self.assertEqual(len(topic1.sent), 2)
        self.assertEqual(len(topic2.sent), 2)

        self.assertListEqual(topic1.sent, [3, 5])
        self.assertListEqual(topic2.sent, [4, 2])

    def test_tf_globals(self):
        self.assertEquals(len(config.active_node.global_data), 0)

        @nrp.MapVariable("shared_var", initial_value=2, scope=nrp.GLOBAL)
        @nrp.Neuron2Robot(Topic("/vars/shared1", float))
        def echo_shared_var(t, shared_var):
            shared_var.value = shared_var.value + t
            return shared_var.value

        @nrp.MapVariable("shared_var", scope=nrp.GLOBAL)
        @nrp.Neuron2Robot(Topic("/vars/shared2", float))
        def echo_shared_var2(t, shared_var):
            shared_var.value = shared_var.value - t
            return shared_var.value

        @nrp.MapVariable("shared_var2", initial_value=5, scope=nrp.GLOBAL)
        @nrp.Neuron2Robot(Topic("/vars/shared3", float))
        def echo_shared_var3(t, shared_var2):
            shared_var2.value = shared_var2.value - t
            return shared_var2.value

        nrp.initialize("MyTransferFunctions")

        self.assertEquals(len(config.active_node.global_data), 2)
        self.assertIn("shared_var", config.active_node.global_data)
        self.assertIn("shared_var2", config.active_node.global_data)

        topic1 = echo_shared_var.topic
        topic2 = echo_shared_var2.topic
        topic3 = echo_shared_var3.topic

        config.active_node.run_neuron_to_robot(1)
        config.active_node.run_neuron_to_robot(2)

        self.assertEqual(len(topic1.sent), 2)
        self.assertEqual(len(topic2.sent), 2)
        self.assertEqual(len(topic3.sent), 2)

        self.assertListEqual(topic1.sent, [3, 4])
        self.assertListEqual(topic2.sent, [2, 2])
        self.assertListEqual(topic3.sent, [4, 2])

    def test_tf_globals(self):
        self.assertEquals(len(config.active_node.global_data), 0)

        @nrp.MapVariable("shared_var", initial_value=2, scope=nrp.GLOBAL)
        @nrp.Neuron2Robot(Topic("/vars/shared1", float))
        def echo_shared_var(t, shared_var):
            shared_var.value = shared_var.value + t
            return shared_var.value

        @nrp.MapVariable("shared_var", scope=nrp.GLOBAL)
        @nrp.Neuron2Robot(Topic("/vars/shared2", float))
        def echo_shared_var2(t, shared_var):
            shared_var.value = shared_var.value - t
            return shared_var.value

        @nrp.MapVariable("shared_var2", global_key="second_var", initial_value=5, scope=nrp.GLOBAL)
        @nrp.Neuron2Robot(Topic("/vars/shared3", float))
        def echo_shared_var3(t, shared_var2):
            shared_var2.value = shared_var2.value - t
            return shared_var2.value

        nrp.initialize("MyTransferFunctions")

        self.assertEquals(len(config.active_node.global_data), 2)
        self.assertIn("shared_var", config.active_node.global_data)
        self.assertIn("second_var", config.active_node.global_data)

        topic1 = echo_shared_var.topic
        topic2 = echo_shared_var2.topic
        topic3 = echo_shared_var3.topic

        config.active_node.run_neuron_to_robot(1)
        config.active_node.run_neuron_to_robot(2)

        self.assertEqual(len(topic1.sent), 2)
        self.assertEqual(len(topic2.sent), 2)
        self.assertEqual(len(topic3.sent), 2)

        self.assertListEqual(topic1.sent, [3, 4])
        self.assertListEqual(topic2.sent, [2, 2])
        self.assertListEqual(topic3.sent, [4, 2])

    def test_invalid_missing_param(self):
        with self.assertRaises(Exception):
            @nrp.MapVariable("shared_var_not_available", initial_value=2, scope=nrp.GLOBAL)
            @nrp.Neuron2Robot(Topic("/vars/shared1", float))
            def echo_shared_var(t, shared_var):
                shared_var.value = shared_var.value + t
                return shared_var.value

    def test_invalid_scope(self):
        @nrp.MapVariable("shared_var", initial_value=2, scope="something_invalid")
        @nrp.Neuron2Robot(Topic("/vars/shared1", float))
        def echo_shared_var(t, shared_var):
            shared_var.value = shared_var.value + t
            return shared_var.value

        self.assertRaises(Exception, nrp.initialize, "MyTransferFunctions")


if __name__ == "__main__":
    unittest.main()
