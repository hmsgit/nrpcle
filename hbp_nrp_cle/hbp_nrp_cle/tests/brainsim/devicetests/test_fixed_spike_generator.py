# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
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
from hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator import PyNNFixedSpikeGenerator as FixedSpikeGenerator
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


class TestFixedSpikeGenerator(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.RandomDistribution")
    def test_default_config(self, random_mock, sim_mock):
        dev = FixedSpikeGenerator()
        dev.connect(Mock())
        self.assertTrue(sim_mock().Population.called)
        self.assertTrue(sim_mock().initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock().AllToAllConnector(),
            'weight': random_mock(),
            'delay': None,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock().StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    def test_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'OneToOne'}, receptor_type="inhibitory")
        dev.connect(Mock())
        self.assertTrue(sim_mock().Population.called)
        self.assertTrue(sim_mock().initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock().OneToOneConnector(),
            'weight': 2,
            'delay': 4,
            'source': None,
            'receptor_type': 'inhibitory',
            'synapse_type': sim_mock().StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    def test_connector_no_delays(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 1, 'mode': 'Fixed', 'n': 1})
        dev.connect(Mock())
        self.assertTrue(sim_mock().Population.called)
        self.assertTrue(sim_mock().initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock().FixedNumberPreConnector(),
            'weight': 1,
            'delay': None,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock().StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'AllToAll'}, weight=42)
        dev.connect(Mock())
        self.assertTrue(sim_mock().Population.called)
        self.assertTrue(sim_mock().initialize.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'cm': 1.0,
            'tau_m': 1000.0,
            'tau_refrac': FixedSpikeGenerator.default_parameters['tau_refrac'],
            'v_thresh': -50.0,
            'v_reset': -100.0,
            'v_rest': -100.0,
            'connector': sim_mock().AllToAllConnector(),
            'weight': 42,
            'delay': 4,
            'source': None,
            'receptor_type': 'excitatory',
            'synapse_type': sim_mock().StaticSynapse(),
            'label': None
        })

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    def test_synapse_dynamics(self, sim_mock):
        dev = FixedSpikeGenerator(synapse_type={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        dev.connect(Mock())
        self.assertTrue(sim_mock().Population.called)
        self.assertTrue(sim_mock().initialize.called)
        self.assertIsNotNone(dev._parameters["synapse_type"])
        self.assertTrue(sim_mock().TsodyksMarkramSynapse.called)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    def test_invalid_connector_raises(self, sim_mock):
        connector={'weight': 2, 'delay': 4, 'mode': 'invalid'}
        self.assertRaises(Exception, FixedSpikeGenerator, connector=connector)
        del connector['mode']
        self.assertRaises(Exception, FixedSpikeGenerator, connector=connector)

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator.PyNNFixedSpikeGenerator.sim")
    def test_activation(self, sim_mock):
        dev = FixedSpikeGenerator()
        dev.connect(Mock())

        self.assertTrue(dev.active)

        dev.active = False
        self.assertFalse(dev.active)

        dev.active = True
        self.assertTrue(dev.active)
