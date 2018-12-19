
import unittest
import os
from mock import Mock, patch
from testfixtures import log_capture
import nengo
from hbp_nrp_cle.brainsim.nengo.NengoSimulationState import NengoSimulationState
import hbp_nrp_cle.tf_framework as tf_framework
import hbp_nrp_cle.brainsim.config as brainconfig

__author__ = 'Sebastian Krach'


class NengoSimulationStateTest(unittest.TestCase):
    def setUp(self):
        """
        Initialize the Nengo Simulation State object
        """

        self.sim_state = NengoSimulationState()
        self.assertFalse(self.sim_state.initialized)
        self.sim_factory = Mock()
        self.sim_state.initialize(self.sim_factory)
        self.assertTrue(self.sim_state.initialized)

        self.assertIsNone(self.sim_state.brain_root)

    @patch('hbp_nrp_cle.common.refresh_resources')
    @log_capture('hbp_nrp_cle.brainsim.nengo.NengoSimulationState')
    def testLoadBrain(self, logcapture):
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nengo_brain.py')
        self.sim_state.load_brain(filename)
        self.assertIsNotNone(self.sim_state.brain_root)
        self.assertListEqual(self.sim_state.brain_root.ensembles, [])
        self.assertEquals(len(self.sim_state.brain_root.networks), 1)
        self.assertEquals(self.sim_state.brain_root.networks[0].label, "DummyNetwork")
        logcapture.check('hbp_nrp_cle.brainsim.nengo.NengoSimulationState', 'INFO',
                         'Resetting Nengo simulator')

    @patch('hbp_nrp_cle.common.refresh_resources')
    @log_capture('hbp_nrp_cle.brainsim.nengo.NengoSimulationState')
    def testInitializeSimulator(self, logcapture):
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nengo_brain.py')
        self.sim_state.load_brain(filename)
        self.assertIsNotNone(self.sim_state.brain_root)
        self.assertEquals(self.sim_state.brain_root.networks[0].label, "DummyNetwork")
        logcapture.check('hbp_nrp_cle.brainsim.nengo.NengoSimulationState', 'INFO',
                         'Resetting Nengo simulator')
        self.assertIsNotNone(self.sim_state.simulator)
        self.sim_factory.assert_called_once_with(self.sim_state.brain_root)
        logcapture.check('hbp_nrp_cle.brainsim.nengo.NengoSimulationState', 'INFO',
                         'Initializing new Nengo simulator instance')

    @patch('hbp_nrp_cle.common.refresh_resources')
    @log_capture('hbp_nrp_cle.brainsim.nengo.NengoSimulationState')
    def testSimulatorStateWith(self, logcapture):
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nengo_brain.py')
        self.sim_state.load_brain(filename)
        self.assertIsNotNone(self.sim_state.brain_root)
        self.assertEquals(len(self.sim_state.brain_root.ensembles), 0)

        with self.sim_state:
            nengo.Ensemble(100, 1)

        self.assertEquals(len(self.sim_state.brain_root.ensembles), 1)
        self.assertEquals(self.sim_state.brain_root.ensembles[0].n_neurons, 100)
        self.assertEquals(self.sim_state.brain_root.ensembles[0].dimensions, 1)

    @patch('hbp_nrp_cle.common.refresh_resources')
    @log_capture('hbp_nrp_cle.brainsim.nengo.NengoSimulationState')
    def testDeleteFromModel(self, logcapture):
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nengo_brain.py')
        self.sim_state.load_brain(filename)
        self.assertIsNotNone(self.sim_state.brain_root)
        self.assertEquals(len(self.sim_state.brain_root.nodes), 0)

        with self.sim_state:
            node = nengo.Node()

        self.assertEquals(len(self.sim_state.brain_root.nodes), 1)
        self.assertEquals(self.sim_state.brain_root.nodes[0], node)
        self.sim_state.delete_from_brain(node)
        self.assertEquals(len(self.sim_state.brain_root.nodes), 0)


if __name__ == "__main__":
    unittest.main()
