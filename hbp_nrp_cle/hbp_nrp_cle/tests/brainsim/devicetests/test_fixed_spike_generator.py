from hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator import PyNNFixedSpikeGenerator as FixedSpikeGenerator
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


class TestFixedSpikeGenerator(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.RandomDistribution")
    def test_default_config(self, random_mock, sim_mock):
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
            'weight': random_mock(),
            'delay': None,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'OneToOne'}, receptor_type="inhibitory")
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
            'weight': 2,
            'delay': 4,
            'source': None,
            'receptor_type': 'inhibitory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_connector_no_delays(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 1, 'mode': 'Fixed', 'n': 1})
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
            'weight': 1,
            'delay': None,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'AllToAll'}, weight=42)
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
            'weight': 42,
            'delay': 4,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_synapse_dynamics(self, sim_mock):
        dev = FixedSpikeGenerator(synapse_type={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertIsNotNone(dev._parameters["synapse_type"])
        self.assertTrue(sim_mock.TsodyksMarkramSynapse.called)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.sim")
    def test_invalid_connector_raises(self, sim_mock):
        connector={'weight': 2, 'delay': 4, 'mode': 'invalid'}
        self.assertRaises(Exception, FixedSpikeGenerator, connector=connector)
        del connector['mode']
        self.assertRaises(Exception, FixedSpikeGenerator, connector=connector)
