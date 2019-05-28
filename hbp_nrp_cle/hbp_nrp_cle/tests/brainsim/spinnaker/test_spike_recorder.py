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
from hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerSpikeRecorder import PyNNSpiNNakerSpikeRecorder
from mock import patch, Mock
import numpy as np


class TestSpikeRecorder(unittest.TestCase):

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerSpikeRecorder.live_connections")
    def test_spike_recorder_creates_connection(self, live_connections):
        dev = PyNNSpiNNakerSpikeRecorder()
        population = Mock()
        population.label = "foo"
        dev.connect(population)

        self.assertFalse(dev.spiked)
        self.assertEqual(repr(np.array([[],[]]).T), repr(dev.times))

        dev.refresh(0.0)
        self.assertFalse(dev.spiked)
        self.assertEqual(repr(np.array([[],[]]).T), repr(dev.times))
        dev.finalize_refresh(0.0)

        dev.refresh(0.1)
        dev.finalize_refresh(0.1)

        self.assertFalse(dev.spiked)
        self.assertEqual(repr(np.array([[],[]]).T), repr(dev.times))

        dev._disconnect()

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__PyNNSpiNNakerSpikeRecorder.live_connections")
    def test_spike_recorder_triggers_tf(self, live_connections):
        dev = PyNNSpiNNakerSpikeRecorder()
        population = Mock()
        population.label = "foo"
        dev.connect(population)
        tf = Mock()
        tf.active = True
        tf.should_run.return_value = True
        tf.elapsed_time = 0

        self.assertTrue(live_connections.register_receiver.called)
