
import unittest
from mock import Mock, patch
import nengo
from hbp_nrp_cle.brainsim.nengo.NengoInfo import NengoPopulationInfo
from hbp_nrp_cle.brainsim.nengo.NengoControlAdapter import NengoControlAdapter
__author__ = 'Sebastian Krach'


class NengoControlAdapterTest(unittest.TestCase):
    def setUp(self):
        """
        Initialize the Nengo Simulation State object
        """

        self.sim_state = Mock()
        self.sim_state.load_brain = Mock()
        self.sim_state.initialize = Mock()

        self.adapter = NengoControlAdapter(self.sim_state)

    @patch('hbp_nrp_cle.brainsim.nengo.NengoBrainLoader.setup_access_to_population')
    def testLoadBrain(self, setupMock):
        import hbp_nrp_cle.tf_framework.config as config

        config.brain_root = nengo.Network()
        with config.brain_root:
            nested = nengo.Network()
            with nested:
                ens = nengo.Ensemble(1, 1)
                act = nengo.Ensemble(1, 1)

        self.adapter.load_brain("filename.py")
        self.sim_state.load_brain.assert_called_with("filename.py")

        self.assertEqual(setupMock.call_args[0][0], config.brain_root)
        for arg in setupMock.call_args[0][1:]:
            self.assertIsInstance(arg, NengoPopulationInfo)
            self.assertIn(arg.population, [ens, act])

    @patch('nengo.Simulator')
    def testInitialize(self, simulatorMock):
        self.assertFalse(self.adapter.is_initialized)

        self.adapter.initialize()

        self.assertTrue(self.adapter.is_initialized)

        self.sim_state.initialize.assert_called_once()
        self.assertEqual(simulatorMock.call_count, 0)

        self.sim_state.initialize.call_args[0][0]("brain_dummy")
        simulatorMock.assert_called_once_with("brain_dummy", dt=0.001)

    @patch('nengo.Simulator')
    def testInitializeWithDifferentDT(self, simulatorMock):
        self.assertFalse(self.adapter.is_initialized)

        self.adapter.initialize(dt=1.0)

        self.assertTrue(self.adapter.is_initialized)

        self.sim_state.initialize.assert_called_once()
        self.assertEqual(simulatorMock.call_count, 0)

        self.sim_state.initialize.call_args[0][0]("brain_dummy")
        simulatorMock.assert_called_once_with("brain_dummy", dt=1.0)

    @patch('nengo.Simulator')
    def testRunStep(self, simulatorMock):
        self.assertFalse(self.adapter.is_initialized)
        self.adapter.initialize()
        self.assertTrue(self.adapter.is_initialized)

        self.sim_state.simulator = Mock(dt=0.1)
        self.sim_state.simulator.step = Mock()

        self.adapter.run_step(1000)
        self.assertEquals(self.sim_state.simulator.step.call_count, 10)


if __name__ == "__main__":
    unittest.main()
