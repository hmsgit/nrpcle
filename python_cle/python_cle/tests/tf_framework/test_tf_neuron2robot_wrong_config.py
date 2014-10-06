from python_cle.tf_framework import Facade as nrp
from python_cle.tests.tf_framework.husky import Husky

from python_cle.mocks.robotsim.MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, \
    MockPublishedTopic
from python_cle.mocks.brainsim.MockBrainCommunicationAdapter import MockBrainCommunicationAdapter

import unittest

__author__ = 'GeorgHinkel'


class Neuron2RobotTests(unittest.TestCase):
    def test_parameter_not_parsable_fails(self):
        nrp.start_new_tf_manager()

        # This test fails because the parameter name neuronA cannot be mapped to a GID
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuronA):
            return neuronA.voltage

        self.complete_with_exception(ValueError)

    def test_map_neuron_wrong_parameter_fails(self):
        nrp.start_new_tf_manager()
        try:
            @nrp.MapNeuronParameter("neuronX", [1, 2, 3], nrp.leaky_integrator_exp)
            @nrp.Neuron2Robot(Husky.RightArm.pose)
            def right_arm(t, neuron0):
                return neuron0.voltage

            self.init_adapters()
            self.fail()
        except Exception:
            pass

    def test_map_robot_fails(self):
        nrp.start_new_tf_manager()
        try:
            @nrp.MapRobotParameter("neuronX", Husky.LeftArm.twist)
            @nrp.Neuron2Robot(Husky.RightArm.pose)
            def right_arm(t, neuron0):
                return neuron0.voltage

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
