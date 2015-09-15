from hbp_nrp_cle.tf_framework import _Facade as nrp
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter
import unittest

__author__ = 'GeorgHinkel'


class Neuron2RobotTests(unittest.TestCase):
    def test_parameter_not_parsable_fails(self):
        nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because there is no mapping for parameter "neuron1"

        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron1):
            return neuronA.voltage

        self.init_adapters()

        self.assertRaises(Exception, nrp.initialize)

    def test_map_neuron_wrong_parameter_fails(self):
        nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because "neuronX" cannot be mapped to a parameter
        with self.assertRaises(Exception):
            @nrp.MapSpikeSink("neuronX", [1, 2, 3], nrp.leaky_integrator_exp)
            @nrp.Neuron2Robot(Husky.RightArm.pose)
            def right_arm(t):
                return neuron0.voltage

    def init_adapters(self):
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

if __name__ == "__main__":
    unittest.main()
