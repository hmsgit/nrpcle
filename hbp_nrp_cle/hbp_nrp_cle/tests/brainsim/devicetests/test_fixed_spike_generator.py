from hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator import PyNNFixedSpikeGenerator as FixedSpikeGenerator
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


class TestFixedSpikeGenerator(unittest.TestCase):

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_default_config(self, sim_mock):
        dev = FixedSpikeGenerator()
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock.AllToAllConnector(),
            'weights': sim_mock.RandomDistribution(),
            'delays': sim_mock.RandomDistribution(),
            'source': None,
            'target': 'excitatory',
            'synapse_dynamics': None,
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weights': 2, 'delays': 4, 'mode': 'OneToOne'})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock.OneToOneConnector(),
            'weights': 2,
            'delays': 4,
            'source': None,
            'target': 'excitatory',
            'synapse_dynamics': None,
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_connector_no_delays(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weights': 1, 'mode': 'Fixed', 'n': 1})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock.FixedNumberPreConnector(),
            'weights': 1,
            'delays': sim_mock.RandomDistribution(),
            'source': None,
            'target': 'excitatory',
            'synapse_dynamics': None,
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weights': 2, 'delays': 4, 'mode': 'AllToAll'}, weights=42)
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock.AllToAllConnector(),
            'weights': 42,
            'delays': 4,
            'source': None,
            'target': 'excitatory',
            'synapse_dynamics': None,
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_synapse_dynamics(self, sim_mock):
        dev = FixedSpikeGenerator(synapse_dynamics={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertIsNotNone(dev._parameters["synapse_dynamics"])
        self.assertTrue(sim_mock.TsodyksMarkramMechanism.called)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_invalid_connector_raises(self, sim_mock):
        connector={'weights': 2, 'delays': 4, 'mode': 'invalid'}
        dev = FixedSpikeGenerator(connector=connector)
        self.assertRaises(Exception, dev.connect, Mock())
        del connector['mode']
        self.assertRaises(Exception, dev.connect, Mock())
