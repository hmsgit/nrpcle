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