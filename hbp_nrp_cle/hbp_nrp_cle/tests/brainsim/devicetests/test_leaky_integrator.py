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
from hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegratorTypes import PyNNLeakyIntegratorAlpha as LeakyIntegrator
import unittest
from mock import patch

__author__ = 'Georg Hinkel'


class TestLeakyIntegrator(unittest.TestCase):

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator.sim")
    def test_default_config(self, sim_mock):
        dev = LeakyIntegrator()
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'v_thresh': float('inf'),
            'cm': 1.0,
            'tau_m': 10.0,
            'tau_syn_E': 2.,
            'tau_syn_I': 2.,
            'v_rest': 0.0,
            'v_reset': 0.0,
            'tau_refrac': 0.1,
            'i_offset': 0.0,
            'connector': sim_mock.AllToAllConnector(),
            'weight': 0.01,
            'delay': 0.1,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator.sim")
    def test_connector(self, sim_mock):
        dev = LeakyIntegrator(connector={'weight': 2, 'delay': 4, 'mode': 'OneToOne'})
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'v_thresh': float('inf'),
            'cm': 1.0,
            'tau_m': 10.0,
            'tau_syn_E': 2.,
            'tau_syn_I': 2.,
            'v_rest': 0.0,
            'v_reset': 0.0,
            'tau_refrac': 0.1,
            'i_offset': 0.0,
            'connector': sim_mock.OneToOneConnector(),
            'weight': 2,
            'delay': 4,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator.sim")
    def test_connector_no_delays(self, sim_mock):
        dev = LeakyIntegrator(connector={'weight': 1, 'mode': 'Fixed', 'n': 1})
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'v_thresh': float('inf'),
            'cm': 1.0,
            'tau_m': 10.0,
            'tau_syn_E': 2.,
            'tau_syn_I': 2.,
            'v_rest': 0.0,
            'v_reset': 0.0,
            'tau_refrac': 0.1,
            'i_offset': 0.0,
            'connector': sim_mock.FixedNumberPreConnector(),
            'weight': 1,
            'delay': 0.1,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator.sim")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = LeakyIntegrator(connector={'weight': 2, 'delay': 4, 'mode': 'AllToAll'}, weight=42)
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertDictEqual(dev._parameters, {
            'v_thresh': float('inf'),
            'cm': 1.0,
            'tau_m': 10.0,
            'tau_syn_E': 2.,
            'tau_syn_I': 2.,
            'v_rest': 0.0,
            'v_reset': 0.0,
            'tau_refrac': 0.1,
            'i_offset': 0.0,
            'connector': sim_mock.AllToAllConnector(),
            'weight': 42,
            'delay': 4,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock.StaticSynapse(),
            'label': None,
            'rng': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator.sim")
    def test_synapse_dynamics(self, sim_mock):
        dev = LeakyIntegrator(synapse_type={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        self.assertTrue(sim_mock.Population.called)
        self.assertTrue(sim_mock.initialize.called)
        self.assertIsNotNone(dev._parameters["synapse_type"])
        self.assertTrue(sim_mock.TsodyksMarkramSynapse.called)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator.sim")
    def test_invalid_connector_raises(self, sim_mock):
        connector={'weight': 2, 'delay': 4, 'mode': 'invalid'}
        self.assertRaises(Exception, LeakyIntegrator, connector=connector)
        del connector['mode']
        self.assertRaises(Exception, LeakyIntegrator, connector=connector)
