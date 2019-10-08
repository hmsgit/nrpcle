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
from hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator import NestFixedSpikeGenerator as FixedSpikeGenerator
import unittest
from mock import patch, Mock


class TestNestFixedSpikeGenerator(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_default_config(self, sim_mock):
        dev = FixedSpikeGenerator()
        dev.connect(Mock())
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.SetStatus.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'C_m': 1000.0,
            'tau_m': 1000.0,
            't_ref': FixedSpikeGenerator.default_parameters['t_ref'],
            'V_th': -50.0,
            'V_reset': -100.0,
            'E_L': -100.0,
            'connector': 'all_to_all',
            'weight': 0.15,
            'delay': 0.1,
            'receptor_type': 'excitatory',
            'synapse_type': {'model': 'static_synapse'}
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'OneToOne'}, receptor_type="inhibitory")
        dev.connect(Mock())
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.SetStatus.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'C_m': 1000.0,
            'tau_m': 1000.0,
            't_ref': FixedSpikeGenerator.default_parameters['t_ref'],
            'V_th': -50.0,
            'V_reset': -100.0,
            'E_L': -100.0,
            'connector': 'one_to_one',
            'weight': -2,
            'delay': 4,
            'receptor_type': 'inhibitory',
            'synapse_type': {'model': 'static_synapse'}
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_connector_no_delays(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 1, 'mode': 'Fixed', 'n': 1})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.SetStatus.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'C_m': 1000.0,
            'tau_m': 1000.0,
            't_ref': FixedSpikeGenerator.default_parameters['t_ref'],
            'V_th': -50.0,
            'V_reset': -100.0,
            'E_L': -100.0,
            'connector': {'rule': 'fixed_indegree', 'indegree': 1},
            'weight': 1,
            'delay': 0.1,
            'receptor_type': 'excitatory',
            'synapse_type': {'model': 'static_synapse'}
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_manual_weight_overrides_connector(self, sim_mock):
        dev = FixedSpikeGenerator(connector={'weight': 2, 'delay': 4, 'mode': 'AllToAll'}, weight=42)
        dev.connect(Mock())
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.SetStatus.called)
        self.assertDictEqual(dev._parameters, {
            'initial_rate': 0.0,
            'C_m': 1000.0,
            'tau_m': 1000.0,
            't_ref': FixedSpikeGenerator.default_parameters['t_ref'],
            'V_th': -50.0,
            'V_reset': -100.0,
            'E_L': -100.0,
            'connector': 'all_to_all',
            'weight': 42,
            'delay': 4,
            'receptor_type': 'excitatory',
            'synapse_type': {'model': 'static_synapse'}
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_synapse_dynamics(self, sim_mock):
        dev = FixedSpikeGenerator(synapse_type={'type': 'TsodyksMarkram', 'U': 0, 'tau_rec': 8, 'tau_facil': 15})
        dev.connect(Mock())
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.SetStatus.called)
        self.assertDictEqual(dev._parameters["synapse_type"], {
            'model': 'tsodyks_synapse',
            'U': 0,
            'tau_rec': 8,
            'tau_fac': 15
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_invalid_connector_raises(self, _):
        connector = {'weight': 2, 'delay': 4, 'mode': 'invalid'}
        self.assertRaises(Exception, FixedSpikeGenerator, connector=connector)
        del connector['mode']
        self.assertRaises(Exception, FixedSpikeGenerator, connector=connector)

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestFixedSpikeGenerator.nest")
    def test_activation(self, _):
        dev = FixedSpikeGenerator()
        dev.connect(Mock())

        self.assertTrue(dev.active)

        dev.active = False
        self.assertFalse(dev.active)

        dev.active = True
        self.assertTrue(dev.active)
