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
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connection
import unittest
from mock import patch, Mock


class TestLiveConnection(unittest.TestCase):

    def setUp(self):
        live_connection.shutdown()

    def test_get_port(self):
        self.assertEqual(live_connection.get_port(), 12345)
        self.assertEqual(live_connection.get_port(12346), 12346)
        self.assertEqual(live_connection.get_port(12345), 12347)
        self.assertEqual(live_connection.get_port(), 12348)

    def test_get_register_receiver(self):
        callback = Mock()
        live_connection.register_receiver("foo", callback)
        live_connection.register_receiver("foo", callback)
        self.assertDictEqual(live_connection.receive_callbacks,
                             {
                                 "foo": [callback, callback]
                             })

    def test_get_register_sender(self):
        callback = Mock()
        live_connection.register_sender("foo", callback)
        live_connection.register_sender("foo", callback)
        self.assertDictEqual(live_connection.send_callbacks,
                             {
                                 "foo": [callback, callback]
                             })

    def test_get_register_poisson(self):
        callback = Mock()
        live_connection.register_poisson("foo", callback)
        live_connection.register_poisson("foo", callback)
        self.assertDictEqual(live_connection.poisson_callbacks,
                             {
                                 "foo": [callback, callback]
                             })

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection.spynnaker")
    def test_create_send_connection(self, sim):

        callback = Mock()
        live_connection.register_sender("foo", callback)
        live_connection.register_sender("bar", callback)

        live_connection.create_and_start_connections()
        sim.external_devices.SpynnakerLiveSpikesConnection.assert_called_once_with(
            send_labels=["foo", "bar"], receive_labels=None, local_port=live_connection.SEND_PORT
        )
        connection = sim.external_devices.SpynnakerLiveSpikesConnection.return_value
        connection.add_start_resume_callback.assert_any_call("foo", callback)
        connection.add_start_resume_callback.assert_any_call("bar", callback)

        live_connection.create_and_start_connections()
        sim.external_devices.SpynnakerLiveSpikesConnection.assert_called_once_with(
            send_labels=["foo", "bar"], receive_labels=None, local_port=live_connection.SEND_PORT
        )

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection.spynnaker")
    def test_create_receive_connection(self, sim):

        callback = Mock()
        live_connection.register_receiver("foo", callback)
        live_connection.register_receiver("bar", callback)

        live_connection.create_and_start_connections()
        sim.external_devices.SpynnakerLiveSpikesConnection.assert_called_once_with(
            receive_labels=["foo", "bar"], send_labels=None, local_port=live_connection.RECEIVE_PORT
        )
        connection = sim.external_devices.SpynnakerLiveSpikesConnection.return_value
        connection.add_receive_callback.assert_any_call("foo", callback)
        connection.add_receive_callback.assert_any_call("bar", callback)

        live_connection.create_and_start_connections()
        sim.external_devices.SpynnakerLiveSpikesConnection.assert_called_once_with(
            receive_labels=["foo", "bar"], send_labels=None, local_port=live_connection.RECEIVE_PORT
        )

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection.spynnaker")
    def test_create_poisson_connection(self, sim):

        callback = Mock()
        live_connection.register_poisson("foo", callback)
        live_connection.register_poisson("bar", callback)

        live_connection.create_and_start_connections()
        sim.external_devices.SpynnakerPoissonControlConnection.assert_called_once_with(
            poisson_labels=["foo", "bar"], local_port=live_connection.POISSON_PORT
        )
        connection = sim.external_devices.SpynnakerPoissonControlConnection.return_value
        # add_start_resume would be cleaner but requires access to protected variables for Poisson connections
        connection.add_start_callback.assert_any_call("foo", callback)
        connection.add_start_callback.assert_any_call("bar", callback)

        live_connection.create_and_start_connections()
        sim.external_devices.SpynnakerPoissonControlConnection.assert_called_once_with(
            poisson_labels=["foo", "bar"], local_port=live_connection.POISSON_PORT
        )

if __name__ == "__main__":
    unittest.main()