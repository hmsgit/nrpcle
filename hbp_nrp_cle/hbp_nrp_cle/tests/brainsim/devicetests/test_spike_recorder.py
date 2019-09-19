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
from hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder import PyNNNestSpikeRecorder as SpikeRecorder
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


class TestSpikeRecorder(unittest.TestCase):

    def setUp(self):
        pass

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup.nest")
    def test_default_config(self, nest_mock):
        dev = SpikeRecorder()
        neurons = Mock()
        neurons.__iter__ = Mock(return_value = iter([1]))
        neurons.grandparent = Mock()
        populationView = Mock(recorder=Mock())
        neurons.grandparent.__getitem__ = populationView
        neurons.grandparent.all_cells = [1]
        dev.connect(neurons)
        recorder_device = neurons.grandparent.recorder
        recorder_device.reset.assert_called_once_with()
        populationView().record.assert_called_once_with("spikes", to_file=False)
        nest_mock.SetStatus.assert_any_call(recorder_device._spike_detector.device, {"to_memory": True})
        nest_mock.SetStatus.assert_any_call(recorder_device._spike_detector.device, {"to_file": False})
        self.assertDictEqual(dev._parameters, {
            'use_ids': True
        })

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup.nest")
    def test_use_indices_config(self, nest_mock):
        dev = SpikeRecorder(use_ids = False)
        neurons = Mock()
        neurons.__iter__ = Mock(return_value = iter([1]))
        neurons.grandparent = Mock()
        populationView = Mock()
        neurons.grandparent.__getitem__ = populationView
        neurons.grandparent.all_cells = [1]
        dev.connect(neurons)
        recorder_device = neurons.grandparent.recorder
        recorder_device.reset.assert_called_once_with()
        populationView().record.assert_called_once_with("spikes", to_file=False)
        nest_mock.SetStatus.assert_any_call(recorder_device._spike_detector.device, {"to_memory": True})
        nest_mock.SetStatus.assert_any_call(recorder_device._spike_detector.device, {"to_file": False})
        self.assertDictEqual(dev._parameters, {
            'use_ids': False
        })

    @staticmethod
    def id_to_index(index):
        if index == 9:
            return 1
        elif index == 10:
            return 2
        else:
            raise IndexError()

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup.nest")
    def test_default_refresh_config(self, nest_mock):
        dev = SpikeRecorder()
        neurons = Mock()
        neurons.__iter__ = Mock(return_value = iter([]))
        dev.connect(neurons)
        nest_mock.GetStatus.return_value = [{
            'times': [0, 8, 15],
            'senders': [8, 9, 10]
        }]

        neurons.id_to_index = TestSpikeRecorder.id_to_index
        dev.refresh(0.0)

        spikes = dev.times
        self.assertEqual(3, len(spikes))
        print spikes
        self.assertEqual(8.0, spikes[0][0])

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup.nest")
    def test_use_indices_refresh_config(self, nest_mock):
        dev = SpikeRecorder(use_ids = False)
        neurons = Mock()
        neurons.__iter__ = Mock(return_value = iter([]))
        dev.connect(neurons)
        nest_mock.GetStatus.return_value = [{
            'times': [0, 8, 15],
            'senders': [8, 9, 10]
        }]

        neurons.id_to_index = TestSpikeRecorder.id_to_index
        dev.refresh(0.0)

        spikes = dev.times
        self.assertEqual(2, len(spikes))
        print spikes
        self.assertEqual(1.0, spikes[0][0])
