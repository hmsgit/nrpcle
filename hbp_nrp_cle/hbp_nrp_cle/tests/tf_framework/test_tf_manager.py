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
This module contains test cases for the TransferFunctionManager component
"""

__author__ = 'Georg Hinkel'

from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import MockBrainCommunicationAdapter
from hbp_nrp_cle.mocks.robotsim._MockRobotCommunicationAdapter import MockRobotCommunicationAdapter
from hbp_nrp_cle.tf_framework._TransferFunctionManager import TransferFunctionManager
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.tests.husky import Husky

import unittest
import mock


class TestBrain(object):
    @property
    def circuit(self):
        return mock.MagicMock()

    @property
    def sensors(self):
        return mock.MagicMock()

    @property
    def actors(self):
        return mock.MagicMock()

class TestTransferFunctionManager(unittest.TestCase):

    def setUp(self):
        self.bcm = MockBrainCommunicationAdapter()
        self.rcm = MockRobotCommunicationAdapter()
        nrp.start_new_tf_manager()
        nrp.config.brain_root = TestBrain()
        self.tfm = nrp.config.active_node

    def test_get_transfer_functions(self):
        tf_name = "tf_name"
        tf_src = "def tf_name(self): pass"
        tf_error = Exception()

        nrp.set_flawed_transfer_function(tf_src, tf_name, tf_error)

        # get also flawed tfs
        tfs = self.tfm.transfer_functions(flawed=True)

        self.assertEqual(tfs[0].name, tf_name)

        # get only proper tfs
        tfs = self.tfm.transfer_functions(flawed=False)

        self.assertEqual(len(tfs), 0)

    def test_setting_brainsim_adapter(self):
        self.tfm.robot_adapter = self.rcm

        self.assertRaises(Exception, TransferFunctionManager.brain_adapter.setter, self.tfm, None)
        self.assertRaises(Exception, TransferFunctionManager.brain_adapter.setter, self.tfm, self.rcm)
        self.assertRaises(Exception, self.tfm.initialize, "tfnode")
        self.tfm.brain_adapter = self.bcm
        self.tfm.initialize("tfnode")
        self.assertRaises(Exception, TransferFunctionManager.brain_adapter.setter, self.tfm, MockBrainCommunicationAdapter())

    def test_setting_robotsim_adapter(self):
        self.tfm.brain_adapter = self.bcm

        self.assertRaises(Exception, TransferFunctionManager.robot_adapter.setter, self.tfm, None)
        self.assertRaises(Exception, TransferFunctionManager.robot_adapter.setter, self.tfm, self.bcm)
        self.assertRaises(Exception, self.tfm.initialize, "tfnode")
        self.tfm.robot_adapter = self.rcm
        self.tfm.initialize("tfnode")
        self.assertRaises(Exception, TransferFunctionManager.robot_adapter.setter, self.tfm, MockRobotCommunicationAdapter())

    def test_hard_reset_brain(self):
        self.tfm.robot_adapter = self.rcm
        self.tfm.hard_reset_brain_devices()
        self.tfm.brain_adapter = self.bcm

        @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
        @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
        @nrp.Robot2Neuron()
        def camera_trans(t, camera, device):
            pass

        self.tfm.initialize("tfnode")
        self.assertEqual(1, len(self.bcm.detector_devices))
        self.tfm.hard_reset_brain_devices()
        self.assertEqual(2, len(self.bcm.detector_devices))

    def test_hard_reset_robot(self):
        self.tfm.brain_adapter = self.bcm
        self.tfm.hard_reset_robot_devices()
        self.tfm.robot_adapter = self.rcm

        @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
        @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
        @nrp.Robot2Neuron()
        def camera_trans(t, camera, device):
            pass

        self.tfm.initialize("tfnode")
        self.assertEqual(1, len(self.rcm.subscribed_topics))
        self.tfm.hard_reset_brain_devices()

    def test_hard_reset_brain_devices_wrong_mapping(self):
        """
        Any exception thrown resetting brain connection mappings should raise an
        explicit BrainParameterException
        """

        self.tfm.robot_adapter = self.rcm
        self.tfm.hard_reset_brain_devices()
        self.tfm.brain_adapter = self.bcm

        @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
        @nrp.Robot2Neuron()
        def camera_trans(t, device):
            pass

        self.tfm.initialize("tfnode")

        camera_trans.device.spec.create_adapter = lambda: Exception()

        with self.assertRaises(nrp.BrainParameterException) as cm:
            self.tfm.hard_reset_brain_devices()

        self.assertEqual(cm.exception.message, "Cannot map parameter 'device' in transfer function 'camera_trans'")

    def test_shutdown(self):

        self.tfm.robot_adapter = self.rcm
        self.tfm.brain_adapter = self.bcm
        # add tf
        @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
        @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
        @nrp.Robot2Neuron()
        def camera_trans(t, camera, device):
            pass

        # add flawed tf
        tf_name = "tf_name"
        tf_src = "def tf_name(self): pass"
        tf_error = Exception()
        nrp.set_flawed_transfer_function(tf_src, tf_name, tf_error)

        self.tfm.initialize("tfnode")

        self.tfm.shutdown()

        self.assertEqual(len(self.tfm.n2r), 0)
        self.assertEqual(len(self.tfm.r2n), 0)
        self.assertEqual(len(self.tfm.flawed), 0)
        self.assertEqual(len(self.tfm.global_data), 0)
        self.assertFalse(self.tfm.initialized)

    def test_activate_transfer_function(self):

        self.tfm.robot_adapter = self.rcm
        self.tfm.hard_reset_brain_devices()
        self.tfm.brain_adapter = self.bcm

        @nrp.MapSpikeSink("device", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
        @nrp.Neuron2Robot()
        def camera_trans(t, device):
            pass

        @nrp.MapSpikeSource("spike_source", nrp.brain.sensors[0], nrp.poisson)
        @nrp.Robot2Neuron()
        def robot_2_neuron_tf(t, spike_source):
            pass

        #robot_2_neuron_tf.spike_source.spec.activate
        self.tfm.initialize("tfnode")

        camera_tf = self.tfm.n2r[0]
        n_2_r_tf = self.tfm.r2n[0]

        # tf is initialized as active
        self.assertTrue(camera_tf.active)
        self.assertTrue(n_2_r_tf.active)

        # wrong parameter type
        self.tfm.activate_tf(camera_tf, "False")
        self.assertTrue(camera_tf.active)  # no change

        # deactivate tf
        self.tfm.activate_tf(camera_tf, False)
        self.assertFalse(camera_tf.active)

        self.tfm.activate_tf(n_2_r_tf, False)
        self.assertFalse(n_2_r_tf.active)

        # double deactivation
        self.tfm.activate_tf(camera_tf, False)
        self.assertFalse(camera_tf.active)

        self.tfm.activate_tf(n_2_r_tf, False)
        self.assertFalse(n_2_r_tf.active)

        # activate tf
        self.tfm.activate_tf(camera_tf, True)
        self.assertTrue(camera_tf.active)

        self.tfm.activate_tf(n_2_r_tf, True)
        self.assertTrue(n_2_r_tf.active)

        # double activation
        self.tfm.activate_tf(camera_tf, True)
        self.assertTrue(camera_tf.active)

        self.tfm.activate_tf(n_2_r_tf, True)
        self.assertTrue(n_2_r_tf.active)
