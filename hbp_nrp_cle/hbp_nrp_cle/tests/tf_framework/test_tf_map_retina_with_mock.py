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

    @patch('pyretina.InterfaceNEST')
    def test_tf_map_retina(self, mock_retina_interface_nest):
        mock_retina_interface_nest.return_value = nrp.PyRetinaWrapper('fakeretina')
        @nrp.MapRetina("retina", "someConfig.py")
        @nrp.Neuron2Robot(Topic("/vars/shared1", list))
        def echo_shared_var(t, retina):
            ret = [ isinstance(retina, nrp.PyRetinaWrapper),
                    hasattr(retina, 'update'),
                    hasattr(retina, 'reset'),
                    hasattr(retina, 'getValue') ]
            return ret

        nrp.initialize("MyTransferFunctions")

        topic1 = echo_shared_var.topic
        config.active_node.run_neuron_to_robot(1)
        for assertion in topic1.sent:
            self.assertTrue(assertion)

    @patch('pyretina.InterfaceNEST')
    def test_tf_map_retina_not_found(self, mock_retina_interface_nest):
        mock_retina_interface_nest.side_effect = ValueError('no such file')
        @nrp.MapRetina("retina", "dontexist.py")
        @nrp.Neuron2Robot(Topic("/vars/shared1", list))
        def echo_shared_var(t, retina):
            return 1

        self.assertRaises(ValueError, nrp.initialize, "MyTransferFunctions")


if __name__ == "__main__":
    unittest.main()
