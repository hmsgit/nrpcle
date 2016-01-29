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