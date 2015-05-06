"""
ROSCLEClient unit test
"""

import rospy
import logging
from hbp_nrp_cle.cle.ROSCLEState import ROSCLEState
from hbp_nrp_cle.cle import ROSCLEClient
from mock import patch, Mock, MagicMock, DEFAULT, NonCallableMagicMock
from testfixtures import log_capture
import unittest


__author__ = 'HBP NRP software team'


class TestROSCLEClient(unittest.TestCase):

    LOGGER_NAME = ROSCLEClient.__name__

    @patch('rospy.ServiceProxy')
    def test_constructor(self, ServiceProxyMock):
        ServiceProxyMocks = [MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()]
        ServiceProxyMock.side_effect = ServiceProxyMocks
        client = ROSCLEClient.ROSCLEClient()
        for mocks in ServiceProxyMocks:
            mocks.wait_for_service.assert_called_with(timeout=180)

    @patch('rospy.ServiceProxy')
    def test_constructor_timeout(self, ServiceProxyMock):
        ServiceProxyMocks = [MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()]
        ServiceProxyMocks[2].wait_for_service.side_effect = rospy.ROSException()
        ServiceProxyMock.side_effect = ServiceProxyMocks
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client = ROSCLEClient.ROSCLEClient()
        ServiceProxyMocks[0].wait_for_service.assert_called_with(timeout=180)
        ServiceProxyMocks[1].wait_for_service.assert_called_with(timeout=180)
        ServiceProxyMocks[2].wait_for_service.assert_called_with(timeout=180)
        self.assertEqual(len(ServiceProxyMocks[3].wait_for_service.mock_calls), 0)
        self.assertEqual(len(ServiceProxyMocks[4].wait_for_service.mock_calls), 0)

    @patch('rospy.ServiceProxy')
    def test_start_pause_reset(self, ServiceProxyMock):
        ServiceProxyMocks = [MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()]
        ServiceProxyMock.side_effect = ServiceProxyMocks
        client = ROSCLEClient.ROSCLEClient()
        client.start()
        ServiceProxyMocks[0].assert_called_with()
        self.assertEqual(len(ServiceProxyMocks[1].mock_calls), 1)
        self.assertEqual(len(ServiceProxyMocks[2].mock_calls), 1)
        self.assertEqual(len(ServiceProxyMocks[3].mock_calls), 1)
        self.assertEqual(len(ServiceProxyMocks[4].mock_calls), 1)
        client.pause()
        ServiceProxyMocks[1].assert_called_with()
        self.assertEqual(len(ServiceProxyMocks[0].mock_calls), 2)
        self.assertEqual(len(ServiceProxyMocks[2].mock_calls), 1)
        self.assertEqual(len(ServiceProxyMocks[3].mock_calls), 1)
        self.assertEqual(len(ServiceProxyMocks[4].mock_calls), 1)
        client.reset()
        ServiceProxyMocks[3].assert_called_with()
        self.assertEqual(len(ServiceProxyMocks[0].mock_calls), 2)
        self.assertEqual(len(ServiceProxyMocks[1].mock_calls), 2)
        self.assertEqual(len(ServiceProxyMocks[2].mock_calls), 1)
        self.assertEqual(len(ServiceProxyMocks[4].mock_calls), 1)

    @patch('rospy.ServiceProxy')
    def test_stop(self, ServiceProxyMock):
        ServiceProxyMocks = [MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()]
        ServiceProxyMock.side_effect = ServiceProxyMocks
        client = ROSCLEClient.ROSCLEClient()
        client.start()
        ServiceProxyMocks[0].assert_called_with()
        client.stop()
        ServiceProxyMocks[2].assert_called_with()
        # After a stop, nothing else can be called:
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.start()
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.pause()
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.stop()
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.reset()

        # get state can still answer (with a warning though)
        assert (ROSCLEState.STOPPED == client.get_simulation_state())
