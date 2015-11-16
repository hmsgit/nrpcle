from mock import patch, PropertyMock, Mock, MagicMock
from rospy import ROSInterruptException, ServiceException, TransportException
from hbp_nrp_cle.robotsim.AsynchronousServiceProxy import AsynchonousRospyServiceProxy
import unittest
from testfixtures import LogCapture
from threading import Event
import time

__author__ = 'Sebastian Krach'

class TestAsynchronousRospyProxy(unittest.TestCase):

    @patch('hbp_nrp_cle.robotsim.AsynchronousServiceProxy.ServiceProxy')
    def setUp(self, proxy_mock):
        self.__rospy_proxy = MagicMock(transport = None)
        proxy_mock.return_value = self.__rospy_proxy
        test_class = object()
        self.__async_proxy = AsynchonousRospyServiceProxy("test/service", test_class,
                                                          persistent=True)

    def tearDown(self):
        self.__async_proxy.close()

    @patch('hbp_nrp_cle.robotsim.AsynchronousServiceProxy.ServiceProxy')
    def test_create_asynchronous_mock(self, proxy_mock):
        test_class = object()
        proxy = AsynchonousRospyServiceProxy("test/service", test_class, persistent=True)
        proxy_mock.assert_called_with("test/service", test_class, True, None)

    def test_close(self):
        self.assertFalse(self.__rospy_proxy.close.called)
        self.__async_proxy.close()
        self.__rospy_proxy.close.assert_called_once_with()

    def test_call_once(self):
        self.assertEqual(self.__rospy_proxy.transport, None)
        self.__rospy_proxy.call.return_value = "foo"
        res_future = self.__async_proxy(1, 2, 3)
        self.__rospy_proxy.call.assert_called_once_with(1, 2, 3)
        self.assertTrue(res_future.done())
        self.assertEqual(res_future.result(), "foo")

    @patch('hbp_nrp_cle.robotsim.AsynchronousServiceProxy.args_kwds_to_message')
    def test_initialized_call(self, aktm_mock):
        aktm_mock.return_value = "ros-message"
        transport_mock = MagicMock()
        transport_mock.receive_once.return_value = ("return-value", )
        self.__rospy_proxy.transport = transport_mock
        self.__rospy_proxy.request_class = "service-class"
        result = self.__async_proxy(1, 2, 3)
        aktm_mock.assert_called_once_with("service-class", (1, 2, 3), {})
        self.assertEqual(result.result(), "return-value")

        delay_event = Event()

        def receive_once_delay(*args, **kwargs):
            delay_event.wait()
            return ("blocking_return", )

        transport_mock.receive_once.side_effect = receive_once_delay
        result = self.__async_proxy(1, 2, 3)
        time.sleep(.5)
        self.assertTrue(result.running())
        self.assertFalse(result.done())

        delay_event.set()
        time.sleep(.5)
        self.assertTrue(result.done())
        self.assertEqual(result.result(), "blocking_return")

    def test_unsuitable_response_length(self):
        transport_mock = MagicMock()
        transport_mock.receive_once.return_value = ()
        self.__rospy_proxy.transport = transport_mock
        result = self.__async_proxy(1, 2, 3)
        self.assertRaises(ServiceException, result.result)

        transport_mock.receive_once.return_value = (1, 2, 3)
        result = self.__async_proxy(1, 2, 3)
        self.assertRaises(ServiceException, result.result)

    def test_call_twice(self):
        delay_event = Event()

        def receive_once_delay(*args, **kwargs):
            delay_event.wait()
            return ("blocking_return", )

        transport_mock = MagicMock()
        self.__rospy_proxy.transport = transport_mock
        transport_mock.receive_once.side_effect = receive_once_delay
        result = self.__async_proxy(1, 2, 3)
        time.sleep(.1)
        self.assertRaises(ServiceException, self.__async_proxy, 3, 4)
        self.assertTrue(result.running())
        self.assertFalse(result.done())
        delay_event.set()
        time.sleep(.1)
        self.assertTrue(result.done())
        self.assertEqual(result.result(), "blocking_return")

    @patch('hbp_nrp_cle.robotsim.AsynchronousServiceProxy.is_shutdown')
    def test_transport_exception(self, shutdown_mock):
        transport_mock = MagicMock()
        self.__rospy_proxy.transport = transport_mock
        transport_mock.receive_once.side_effect = TransportException("foo exception")
        shutdown_mock.return_value = False
        result = self.__async_proxy(1, 2, 3)
        self.assertRaises(ServiceException, result.result)

        shutdown_mock.return_value = True
        result = self.__async_proxy(1, 2, 3)
        self.assertRaises(ROSInterruptException, result.result)











