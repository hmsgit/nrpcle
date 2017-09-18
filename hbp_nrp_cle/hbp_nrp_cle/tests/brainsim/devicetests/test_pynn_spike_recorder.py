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
from hbp_nrp_cle.brainsim.pynn.devices.__PyNNSpikeRecorder import PyNNSpikeRecorder as SpikeRecorder
import unittest
from mock import MagicMock, Mock

__author__ = 'Georg Hinkel'


class TestSpikeRecorder(unittest.TestCase):

    def setUp(self):
        pass

    def test_default_config(self):
        dev = SpikeRecorder()
        neurons = Mock()
        dev.connect(neurons)
        neurons.recorder.reset.assert_called_once_with()
        neurons.record.assert_called_once_with("spikes", to_file=False)
        self.assertDictEqual(dev._parameters, {
            'use_ids': True
        })

    def test_use_indices_config(self):
        dev = SpikeRecorder(use_ids = False)
        neurons = Mock()
        dev.connect(neurons)
        neurons.recorder.reset.assert_called_once_with()
        neurons.record.assert_called_once_with("spikes", to_file=False)
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

    def test_default_refresh_config(self,):
        dev = SpikeRecorder()
        neurons = MagicMock()
        dev.connect(neurons)

        train = MagicMock()
        train.__len__.return_value = 3
        neurons.get_data().segments[-1].spiketrains = [train]
        train.annotations.__getitem__.return_value = [8,9,10]
        neurons.id_to_index = TestSpikeRecorder.id_to_index
        dev.refresh(0.0)

        spikes = dev.times
        self.assertEqual(3, len(spikes))
        print spikes
        self.assertEqual(8.0, spikes[0][0])
