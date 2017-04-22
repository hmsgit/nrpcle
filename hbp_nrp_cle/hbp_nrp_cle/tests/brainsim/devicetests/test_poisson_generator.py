# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
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
            "weight": 0.00015,
            "delay": 0.1,
            "source": None,
            "receptor_type": "excitatory",
            "synapse_type": sim_mock.StaticSynapse(),
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_inhibitory(self, sim_mock):
        dev = PoissonGenerator(receptor_type="inhibitory")
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.AllToAllConnector(),
            "weight": 0.00015,
            "delay": 0.1,
            "source": None,
            "receptor_type": "inhibitory",
            "synapse_type": sim_mock.StaticSynapse(),
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_connector(self, sim_mock):
        dev = PoissonGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'OneToOne'})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.OneToOneConnector(),
            "weight": 2,
            "delay": 4,
            "source": None,
            "receptor_type": "excitatory",
            "synapse_type": sim_mock.StaticSynapse(),
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_connector_no_delays(self, sim_mock):
        dev = PoissonGenerator(connector={'weight': 1, 'mode': 'Fixed', 'n': 1})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.FixedNumberPreConnector(),
            "weight": 1,
            "delay": 0.1,
            "source": None,
            "receptor_type": "excitatory",
            "synapse_type": sim_mock.StaticSynapse(),
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = PoissonGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'AllToAll'}, weight=42)
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            "duration": float("inf"),
            "start": 0.0,
            "rate": 0.0,
            "connector": sim_mock.AllToAllConnector(),
            "weight": 42,
            "delay": 4,
            "source": None,
            "receptor_type": "excitatory",
            "synapse_type": sim_mock.StaticSynapse(),
            "label": None,
            "rng": None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_synapse_dynamics(self, sim_mock):
        dev = PoissonGenerator(synapse_type={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Population.called)
        self.assertFalse(sim_mock.initialize.called)
        self.assertIsNotNone(dev._parameters["synapse_type"])
        self.assertTrue(sim_mock.TsodyksMarkramSynapse.called)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator.sim")
    def test_invalid_connector_raises(self, sim_mock):
        connector={'weight': 2, 'delay': 4, 'mode': 'invalid'}
        self.assertRaises(Exception, PoissonGenerator, connector=connector)
        del connector['mode']
        self.assertRaises(Exception, PoissonGenerator, connector=connector)
