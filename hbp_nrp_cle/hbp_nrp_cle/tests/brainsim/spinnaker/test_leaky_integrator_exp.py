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

import unittest
from hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerLeakyIntegratorExp import PyNNSpiNNakerLeakyIntegratorExp
from mock import patch, Mock
from hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection import reset


class TestLeakyIntegratorExp(unittest.TestCase):
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerLeakyIntegratorExp.sim")
    def test_leaky_interator_exp_corrects_key(self, sim_mock):
        dev = PyNNSpiNNakerLeakyIntegratorExp(key=-1)
        self.assertNotEqual(dev.device_control_key, -1)

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerLeakyIntegratorExp.sim")
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerLeakyIntegratorExp.register_devices")
    def test_leaky_integrator_connect(self, register_device_mock, sim_mock):
        reset()
        dev = PyNNSpiNNakerLeakyIntegratorExp()
        population = Mock()
        population.label = "foo"
        dev.connect(population)
        self.assertTrue(register_device_mock.called)
        self.assertTrue(sim_mock.Projection.called)

        self.assertEqual(dev.device_control_key, 1)
        self.assertEqual(dev.device_control_min_value, 0)
        self.assertGreater(dev.device_control_max_value, 30000)
        self.assertTrue(dev.device_control_uses_payload)
        self.assertEqual(dev.device_control_partition_id, "foo")
        self.assertEqual(dev.device_control_timesteps_between_sending, 10)

        dev2 = PyNNSpiNNakerLeakyIntegratorExp(partition="bar", timesteps=100)
        dev2.connect(population)

        self.assertEqual(dev2.device_control_key, 2)
        self.assertEqual(dev2.device_control_min_value, 0)
        self.assertGreater(dev2.device_control_max_value, 30000)
        self.assertTrue(dev2.device_control_uses_payload)
        self.assertEqual(dev2.device_control_partition_id, "bar")
        self.assertEqual(dev2.device_control_timesteps_between_sending, 100)

        dev.stop_record_voltage()
        dev2.stop_record_voltage()

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerLeakyIntegratorExp.sim")
    def test_leaky_integrator_tf_run(self, sim_mock):
        pass