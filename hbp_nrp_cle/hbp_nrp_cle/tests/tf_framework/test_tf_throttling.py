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
from hbp_nrp_cle.tf_framework import config
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim._MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, \
    MockPublishedTopic
from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import MockBrainCommunicationAdapter
from hbp_nrp_cle.tests.tf_framework.MockBrain import MockPopulation

import unittest
from mock import MagicMock, Mock, patch
import logging


class TestTransferFunctionThrottling(unittest.TestCase):

    def setUp(self):
        nrp.start_new_tf_manager()
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        config.active_node.brain_adapter = brain
        config.active_node.robot_adapter = robot

    def test_throttling_invalid(self):
        with self.assertRaises(Exception):
            @nrp.Neuron2Robot(throttling_rate="slow")
            def invalid_throttling(t):
                pass

    def test_no_throttling(self):
        @nrp.Neuron2Robot(throttling_rate=None)
        def no_throttling(t):
            pass

        no_throttling.run(0.0)
        self.assertFalse(no_throttling.should_run(0.0))
        self.assertTrue(no_throttling.should_run(0.0001))

    def test_throttled_tf(self):
        @nrp.Neuron2Robot(throttling_rate=10)
        def throttled_tf(t):
            pass

        throttled_tf.run(0.0)
        self.assertFalse(throttled_tf.should_run(0.0))
        self.assertFalse(throttled_tf.should_run(0.01))
        self.assertTrue(throttled_tf.should_run(0.1))
        self.assertTrue(throttled_tf.should_run(1.0))

        throttled_tf.run(1.0)
        self.assertFalse(throttled_tf.should_run(0.1))
        self.assertFalse(throttled_tf.should_run(1.0))
        self.assertTrue(throttled_tf.should_run(1.1))
        self.assertTrue(throttled_tf.should_run(42.0))