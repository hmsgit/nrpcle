
import unittest
from mock import Mock, patch, call
from hbp_nrp_cle.brainsim.common import DeviceCommunicationDirection
from hbp_nrp_cle.brainsim.nengo.devices import NengoRawSignalNode

__author__ = 'Sebastian Krach'


class NengoRawSignalDeviceTest(unittest.TestCase):
    def setUp(self):
        """
        Initialize the Nengo Simulation State object
        """
        self.sim_state = Mock()
        self.sim_state.__enter__ = Mock()
        self.sim_state.__exit__ = Mock()
        self.sim_state.delete_from_brain = Mock()

        self.dut_in = NengoRawSignalNode(DeviceCommunicationDirection.IN, self.sim_state)
        self.assertIsNotNone(self.dut_in)
        self.dut_out = NengoRawSignalNode(DeviceCommunicationDirection.OUT, self.sim_state)
        self.assertIsNotNone(self.dut_out)

    def testUninitializedValue(self):
        self.assertIsNone(self.dut_in.value)
        self.assertIsNone(self.dut_out.value)
        self.dut_in.value = 1.0
        self.assertEquals(self.dut_in.value, 1.0)

    @patch('nengo.Connection')
    @patch('nengo.Node')
    def testConnectIn(self, nodeMock, connectionMock):
        neurons = [1, 2, 3, 4]
        self.dut_in.connect(neurons)
        nodeMock.assert_called_once()
        fun = nodeMock.call_args[0][0]
        self.assertDictEqual(nodeMock.call_args[1], {"size_in": 0, "size_out": 4})
        self.dut_in.value = 4.0
        self.assertEquals(fun(0.0), 4.0)

        connectionMock.assert_called_once()
        self.assertListEqual(list(connectionMock.call_args[0]),  [nodeMock(), neurons])

    @patch('nengo.Connection')
    @patch('nengo.Node')
    def testConnectOut(self, nodeMock, connectionMock):
        neurons = [1, 2, 3, 4]
        self.dut_out.connect(neurons)
        nodeMock.assert_called_once()
        fun = nodeMock.call_args[0][0]
        self.assertDictEqual(nodeMock.call_args[1], {"size_in": 4, "size_out": 0})
        fun(0.0, 4.0)
        self.assertEquals(self.dut_out.value, 4.0)

        connectionMock.assert_called_once()
        self.assertListEqual(list(connectionMock.call_args[0]), [neurons, nodeMock()])

    @patch('nengo.Connection')
    @patch('nengo.Node')
    def testDisconnect(self, nodeMock, connectionMock):
        neurons = [1, 2, 3, 4]
        self.dut_out.connect(neurons)
        nodeMock.assert_called_once()
        connectionMock.assert_called_once()
        self.dut_out._disconnect()

        self.assertEquals(self.sim_state.delete_from_brain.call_count, 2)
        expected = [call(nodeMock()), call(connectionMock())]
        self.assertEquals(self.sim_state.delete_from_brain.call_args_list, expected)


if __name__ == "__main__":
    unittest.main()
