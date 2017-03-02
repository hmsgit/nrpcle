# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
from hbp_nrp_cle.mocks.robotsim._MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, \
    MockPublishedTopic
from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import MockBrainCommunicationAdapter
from hbp_nrp_cle.tests.tf_framework.MockBrain import MockPopulation
import sys
import unittest
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.tf_framework import config
from mock import patch
import pyretina

__author__ = 'jacqueskaiser'


class TestTransferFunctionRetina(unittest.TestCase):
    def setUp(self):
        nrp.start_new_tf_manager()
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

        brain.actors = MockPopulation(range(0, 60))
        brain.sensors = MockPopulation(range(45, 645))
        config.brain_root = brain

    @patch('pyretina.Retina')
    @patch('hbp_nrp_cle.tf_framework._GlobalData.MapRetina.create_adapter')
    @patch('hbp_nrp_cle.tf_framework.Neuron2Robot.run')
    def test_tf_map_retina(self, mock_retina_interface_nest, create_adapter_mock, run_mock):
        create_adapter_mock.return_value = pyretina.Retina()
        # The file is existing, though it's not a valid configuration, this is not a problem because
        # we're not executing it
        @nrp.MapRetina("retina", __file__)
        @nrp.Neuron2Robot(Topic("/vars/shared1", list))
        def echo_shared_var(t, retina):
            ret = [ isinstance(retina, pyretina.Retina),
                    hasattr(retina, 'update'),
                    hasattr(retina, 'reset'),
                    hasattr(retina, 'getValue') ]
            return ret

        nrp.initialize("MyTransferFunctions")

        topic1 = echo_shared_var.topic
        config.active_node.run_neuron_to_robot(1)
        for assertion in topic1.sent:
            self.assertTrue(assertion)

    def test_tf_map_retina_not_found(self):
        self.assertRaises(AttributeError, nrp.MapRetina, "retina", "not_existing_file.py")


if __name__ == "__main__":
    unittest.main()
