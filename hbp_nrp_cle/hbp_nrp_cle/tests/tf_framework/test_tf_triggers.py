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


class TestTransferFunctionTriggers(unittest.TestCase):

    def setUp(self):
        nrp.start_new_tf_manager()
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        config.active_node.brain_adapter = brain
        config.active_node.robot_adapter = robot

    def test_triggers_have_correct_type(self):
        with self.assertRaises(Exception):
            @nrp.Neuron2Robot(triggers=42.0)
            def faulty_tf(t):
                pass

    def test_triggers_defaults(self):
        @nrp.Neuron2Robot(triggers=None)
        def no_trigger(t):
            pass

        @nrp.Neuron2Robot(triggers="t")
        def default_trigger(t):
            pass

        self.assertEqual(1, len(no_trigger.triggers))
        self.assertEqual("t", no_trigger.triggers[0])

        self.assertEqual(1, len(default_trigger.triggers))
        self.assertEqual("t", default_trigger.triggers[0])

        config.active_node.initialize("test")

        self.assertEqual(1, len(no_trigger.triggers))
        self.assertEqual("t", no_trigger.triggers[0])

        self.assertEqual(1, len(default_trigger.triggers))
        self.assertEqual("t", default_trigger.triggers[0])

        self.assertIn(no_trigger, config.active_node.n2r)
        self.assertIn(default_trigger, config.active_node.n2r)

    def test_multiple_triggers(self):
        @nrp.MapRobotSubscriber("left", Husky.LeftArm.twist)
        @nrp.MapRobotSubscriber("right", Husky.RightArm.pose)
        @nrp.Neuron2Robot(triggers=["left", "right"])
        def multiple_triggers(t, left, right):
            pass

        @nrp.MapRobotSubscriber("cam", Husky.Eye.camera)
        @nrp.Neuron2Robot(triggers=["cam", "t"])
        def dev_and_t(t, cam):
            pass

        self.assertEqual(2, len(multiple_triggers.triggers))
        self.assertIn("left", multiple_triggers.triggers)
        self.assertIn("right", multiple_triggers.triggers)

        self.assertEqual(2, len(dev_and_t.triggers))
        self.assertIn("t", dev_and_t.triggers)
        self.assertIn("cam", dev_and_t.triggers)

        config.active_node.initialize("test")

        self.assertEqual(2, len(multiple_triggers.triggers))

        self.assertEqual(2, len(dev_and_t.triggers))
        self.assertIn("t", dev_and_t.triggers)

        self.assertNotIn(multiple_triggers, config.active_node.n2r)
        self.assertIn(dev_and_t, config.active_node.n2r)
