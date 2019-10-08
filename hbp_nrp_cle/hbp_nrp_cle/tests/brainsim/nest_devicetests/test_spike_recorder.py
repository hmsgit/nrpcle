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
from hbp_nrp_cle.brainsim.nest.devices.__NestSpikeRecorder import NestSpikeRecorder as SpikeRecorder
import unittest
from mock import Mock, patch


class TestNestSpikeRecorder(unittest.TestCase):

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestSpikeRecorder.nest")
    def test_default_config(self, sim_mock):
        dev = SpikeRecorder()
        neurons = Mock()
        dev.connect(neurons)
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.Connect.called)
        self.assertDictEqual(dev._parameters, {
            'use_ids': True
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestSpikeRecorder.nest")
    def test_use_indices_config(self, sim_mock):
        dev = SpikeRecorder(use_ids=False)
        neurons = Mock()
        dev.connect(neurons)
        self.assertTrue(sim_mock.Create.called)
        self.assertTrue(sim_mock.Connect.called)
        self.assertDictEqual(dev._parameters, {
            'use_ids': False
        })

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestSpikeRecorder.nest")
    def test_default_refresh_config(self, sim_mock):

        dev = SpikeRecorder()
        neurons = Mock()
        dev.connect(neurons)

        sim_mock.configure_mock(**{'GetStatus.return_value': [{'times': [0.1, 0.2, 0.3], 'senders': [8, 9, 10]}]})

        dev.refresh(0.0)

        spikes = dev.times
        self.assertEqual(3, len(spikes))
        print spikes
        self.assertEqual(8.0, spikes[0][0])

    @patch("hbp_nrp_cle.brainsim.nest.devices.__NestSpikeRecorder.nest")
    def test_activation(self, _):
        dev = SpikeRecorder()
        dev.connect(Mock())

        self.assertTrue(dev.active)

        dev.active = False
        self.assertFalse(dev.active)

        dev.active = True
        self.assertTrue(dev.active)
