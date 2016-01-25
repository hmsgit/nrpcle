from hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator import PyNNPoissonSpikeGenerator as PoissonGenerator
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


class TestPoissonGenerator(unittest.TestCase):

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_default_config(self, sim_mock):
        dev = PoissonGenerator()
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.AllToAllConnector(),
            "weights": 0.00015,
            "delays": 0.1,
            "source": None,
            "target": "excitatory",
            "synapse_dynamics": None,
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_connector(self, sim_mock):
        dev = PoissonGenerator(connector={'weights': 2, 'delays': 4, 'mode': 'OneToOne'})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.OneToOneConnector(),
            "weights": 2,
            "delays": 4,
            "source": None,
            "target": "excitatory",
            "synapse_dynamics": None,
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_connector_no_delays(self, sim_mock):
        dev = PoissonGenerator(connector={'weights': 1, 'mode': 'Fixed', 'n': 1})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.FixedNumberPreConnector(),
            "weights": 1,
            "delays": 0.1,
            "source": None,
            "target": "excitatory",
            "synapse_dynamics": None,
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = PoissonGenerator(connector={'weights': 2, 'delays': 4, 'mode': 'AllToAll'}, weights=42)
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.AllToAllConnector(),
            "weights": 42,
            "delays": 4,
            "source": None,
            "target": "excitatory",
            "synapse_dynamics": None,
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_synapse_dynamics(self, sim_mock):
        dev = PoissonGenerator(synapse_dynamics={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertIsNotNone(dev._parameters["synapse_dynamics"])
        self.assertTrue(sim_mock.TsodyksMarkramMechanism.called)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_invalid_connector_raises(self, sim_mock):
        connector={'weights': 2, 'delays': 4, 'mode': 'invalid'}
        self.assertRaises(Exception, PoissonGenerator, connector=connector)
        del connector['mode']
        self.assertRaises(Exception, PoissonGenerator, connector=connector)
