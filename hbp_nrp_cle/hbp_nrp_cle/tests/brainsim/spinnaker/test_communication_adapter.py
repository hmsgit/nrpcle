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
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, \
    IDCSource, IACSource, INCSource, ILeakyIntegratorExp, \
    IFixedSpikeGenerator, ISpikeRecorder, IPoissonSpikeGenerator, IPopulationRate

from hbp_nrp_cle.brainsim.pynn_spiNNaker.PyNNSpiNNakerCommunicationAdapter import PyNNSpiNNakerCommunicationAdapter
from hbp_nrp_cle.brainsim.pynn_spiNNaker.devices import PyNNSpiNNakerACSource, PyNNSpiNNakerDCSource, \
    PyNNSpiNNakerLeakyIntegratorAlpha, PyNNSpiNNakerNCSource, PyNNSpiNNakerFixedSpikeGenerator, \
    PyNNSpiNNakerSpikeRecorder, PyNNSpiNNakerPoissonSpikeGenerator, PyNNSpiNNakerLeakyIntegratorExp, PyNNSpiNNakerPopulationRate
from mock import patch, Mock
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connections

class TestSpinnakerAdapter(unittest.TestCase):
    def test_adapter(self):
        adapter = PyNNSpiNNakerCommunicationAdapter()
        adapter.initialize()
        self.assertEqual(adapter._get_device_type(ILeakyIntegratorAlpha), PyNNSpiNNakerLeakyIntegratorAlpha)
        self.assertEqual(adapter._get_device_type(ILeakyIntegratorExp), PyNNSpiNNakerLeakyIntegratorExp)
        self.assertEqual(adapter._get_device_type(IACSource), PyNNSpiNNakerACSource)
        self.assertEqual(adapter._get_device_type(IDCSource), PyNNSpiNNakerDCSource)
        self.assertEqual(adapter._get_device_type(INCSource), PyNNSpiNNakerNCSource)
        self.assertEqual(adapter._get_device_type(IPoissonSpikeGenerator), PyNNSpiNNakerPoissonSpikeGenerator)
        self.assertEqual(adapter._get_device_type(IFixedSpikeGenerator), PyNNSpiNNakerFixedSpikeGenerator)
        self.assertEqual(adapter._get_device_type(ISpikeRecorder), PyNNSpiNNakerSpikeRecorder)
        self.assertEqual(adapter._get_device_type(IPopulationRate), PyNNSpiNNakerPopulationRate)

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.PyNNSpiNNakerCommunicationAdapter.reset_connection")
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.PyNNSpiNNakerCommunicationAdapter.shutdown")
    def test_init_reset(self, shutdown, reset_connection):
        adapter = PyNNSpiNNakerCommunicationAdapter()

        live_connections.default_receiver = 42
        live_connections.default_sender = 23
        live_connections.default_poisson = 0

        adapter.initialize()
        reset_connection.assert_called_once_with()
        self.assertIsNone(live_connections.default_receiver)
        self.assertIsNone(live_connections.default_sender)
        self.assertIsNone(live_connections.default_poisson)

        sender = Mock()
        receiver = Mock()
        poisson = Mock()
        live_connections.default_sender = sender
        live_connections.default_receiver = receiver
        live_connections.default_poisson = poisson
        adapter.shutdown()

        sender.close.assert_called_once_with()
        receiver.close.assert_called_once_with()
        poisson.close.assert_called_once_with()