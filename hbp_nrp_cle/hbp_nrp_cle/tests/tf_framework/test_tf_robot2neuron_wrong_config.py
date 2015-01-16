from hbp_nrp_cle.tf_framework import _Facade as nrp
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter
import unittest

__author__ = 'GeorgHinkel'


class Robot2NeuronTests(unittest.TestCase):
    def test_map_robot_wrong_parameter(self):
        nrp.start_new_tf_manager()
        try:
            @nrp.MapRobotSubscriber("cameraX", Husky.Eye.camera)
            @nrp.MapSpikeSink("device", nrp.leaky_integrator_alpha)
            @nrp.Robot2Neuron()
            def camera_trans(t, camera, device):
                pass

            self.init_adapters()
            self.fail()
        except Exception:
            pass


    def test_map_neuron_wrong_parameter(self):
        nrp.start_new_tf_manager()
        try:
            @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
            @nrp.MapSpikeSink("deviceX", nrp.leaky_integrator_alpha)
            @nrp.Robot2Neuron()
            def camera_trans(t, camera, device):
                pass

            self.init_adapters()
            self.fail()
        except Exception:
            pass

    def test_neuron_unmapped_fails(self):
        nrp.start_new_tf_manager()
        try:
            @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
            @nrp.Robot2Neuron()
            def camera_trans(t, camera, device):
                pass

            self.init_adapters()
            self.fail()
        except Exception:
            pass

    def test_robot_unmapped_fails(self):
        nrp.start_new_tf_manager()
        try:
            @nrp.MapSpikeSink("device", nrp.leaky_integrator_alpha)
            @nrp.Robot2Neuron()
            def camera_trans(t, camera, device):
                pass

            self.init_adapters()
            self.fail()
        except Exception:
            pass

    def init_adapters(self):
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

    def complete_with_exception(self, exception):

        self.init_adapters()

        self.assertRaises(exception, nrp.initialize, "Unit Test transfer functions")


if __name__ == "__main__":
    unittest.main()