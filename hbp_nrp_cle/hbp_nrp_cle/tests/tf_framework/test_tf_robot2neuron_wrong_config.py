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
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter
import unittest

__author__ = 'GeorgHinkel'


class Robot2NeuronTests(unittest.TestCase):
    def setUp(self):
        nrp.start_new_tf_manager()

    def test_map_robot_wrong_parameter(self):
        with self.assertRaises(Exception):
            @nrp.MapRobotSubscriber("cameraX", Husky.Eye.camera)
            @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
            @nrp.Robot2Neuron()
            def camera_trans(t, camera, device):
                pass

    def test_map_neuron_wrong_parameter(self):
        with self.assertRaises(Exception):
            @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
            @nrp.MapSpikeSink("deviceX", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
            @nrp.Robot2Neuron()
            def camera_trans(t, camera, device):
                pass

    def test_neuron_unmapped_fails(self):
        @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
        @nrp.Robot2Neuron()
        def camera_trans(t, camera, device):
            pass

        self.init_adapters()
        self.assertRaises(Exception, nrp.initialize)

    def test_robot_unmapped_fails(self):
        @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
        @nrp.Robot2Neuron()
        def camera_trans(t, camera, device):
            pass

        self.init_adapters()
        self.assertRaises(Exception, nrp.initialize)

    def init_adapters(self):
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)


if __name__ == "__main__":
    unittest.main()