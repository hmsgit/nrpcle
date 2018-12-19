
import unittest
from mock import Mock, patch
from hbp_nrp_cle.brainsim.common import DeviceCommunicationDirection
from hbp_nrp_cle.brainsim.BrainInterface import IRawSignal

__author__ = 'Sebastian Krach'


class NengoCommunicationAdapterTest(unittest.TestCase):
    def setUp(self):
        """
        Initialize the Nengo Simulation State object
        """

        self.sim_state = Mock()
        from hbp_nrp_cle.brainsim.nengo.NengoCommunicationAdapter import NengoCommunicationAdapter
        self.adapter = NengoCommunicationAdapter(self.sim_state)
        self.device_mock = Mock()

    def testInitialize(self):
        self.assertFalse(self.adapter.is_initialized)
        self.adapter.initialize()
        self.assertTrue(self.adapter.is_initialized)

    @patch('hbp_nrp_cle.brainsim.nengo.NengoCommunicationAdapter.NengoCommunicationAdapter._NengoCommunicationAdapter__device_dict')
    def testRegisterSpikeSource(self, dictmock):
        self.adapter.initialize()
        neurons = 0
        mock_devices = {IRawSignal: self.device_mock}
        dictmock.__getitem__ = Mock()
        dictmock.__getitem__.side_effect = mock_devices.__getitem__
        dictmock.__contains__ = Mock()
        dictmock.__contains__.side_effect = mock_devices.__contains__

        self.device_mock.create_new_device = Mock()
        self.device_mock.create_new_device.return_value = self.device_mock
        self.device_mock.connect = Mock()
        self.adapter.register_spike_source(neurons, IRawSignal)
        self.device_mock.create_new_device.assert_called_once_with(
            neurons,
            direction_kind=DeviceCommunicationDirection.IN,
            nengo_simulation_state=self.sim_state)
        self.device_mock.connect.assert_called_once_with(neurons)

    @patch('hbp_nrp_cle.brainsim.nengo.NengoCommunicationAdapter.NengoCommunicationAdapter._NengoCommunicationAdapter__device_dict')
    def testRegisterSpikeSink(self, dictmock):
        self.adapter.initialize()
        neurons = 0
        mock_devices = {IRawSignal: self.device_mock}
        dictmock.__getitem__ = Mock()
        dictmock.__getitem__.side_effect = mock_devices.__getitem__
        dictmock.__contains__ = Mock()
        dictmock.__contains__.side_effect = mock_devices.__contains__

        self.device_mock.create_new_device = Mock()
        self.device_mock.create_new_device.return_value = self.device_mock
        self.device_mock.connect = Mock()
        self.adapter.register_spike_sink(neurons, IRawSignal)
        self.device_mock.create_new_device.assert_called_once_with(
            neurons,
            direction_kind=DeviceCommunicationDirection.OUT,
            nengo_simulation_state=self.sim_state)
        self.device_mock.connect.assert_called_once_with(neurons)


if __name__ == "__main__":
    unittest.main()
