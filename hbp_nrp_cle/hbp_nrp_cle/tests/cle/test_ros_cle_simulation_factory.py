"""
ROSCLESimulationFactory unit test
"""

import hbp_nrp_cle
from hbp_nrp_cle.cle import ROSCLESimulationFactory
import logging
from mock import patch, MagicMock, Mock
from testfixtures import log_capture
import unittest
import json
import threading
from cle_ros_msgs import srv
import os

__author__ = 'HBP NRP software team'


class TestROSCLESimulationFactory(unittest.TestCase):

    LOGGER_NAME = 'hbp_nrp_cle'

    class MockedServiceRequest(object):
        environment_file = "environment_file.sdf"
        generated_cle_script_file = "path/to/the/generated/cle/script/file"
        gzserver_host = "local"

    @patch('hbp_nrp_cle.cle.ROSCLESimulationFactory.logger')
    def setUp(self, mocked_logger):
        unittest.TestCase.setUp(self)

        # Mock the respective objects and make them available for all tests.
        # Also have a look at the following link:
        # https://docs.python.org/3.5/library/unittest.mock-examples.html
        # #applying-the-same-patch-to-every-test-method
        rospy_patcher = patch('hbp_nrp_cle.cle.ROSCLESimulationFactory.rospy')

        # Ensure that the patchers are cleaned up correctly even in exceptional cases
        # e.g. when an exception was thrown.
        self.addCleanup(rospy_patcher.stop)

        # Start the patch for all tests
        self.__mocked_rospy = rospy_patcher.start()

        self.__ros_cle_simulation_factory = ROSCLESimulationFactory.ROSCLESimulationFactory()
        self.assertEqual(mocked_logger.debug.call_count, 1)
        self.assertEqual(self.__ros_cle_simulation_factory.running_simulation_thread, None)

        self.mocked_service_request = self.MockedServiceRequest()

    def mockThreading(self):
        threading_patcher = patch('hbp_nrp_cle.cle.ROSCLESimulationFactory.threading')
        self.addCleanup(threading_patcher.stop)
        self.__mocked_threading = threading_patcher.start()
        self.__mocked_thread = ROSCLESimulationFactory.threading.Thread()
        self.__mocked_threading.Thread = MagicMock(return_value=self.__mocked_thread)
        self.__ros_cle_simulation_factory.\
            running_simulation_thread = MagicMock(spec=threading.Thread)
        self.__ros_cle_simulation_factory.simulation_initialized_event.wait = MagicMock(return_value=True)

    def tearDown(self):
        # remove all handlers after each test!
        logging.getLogger(self.LOGGER_NAME).handlers = []

    def test_run(self):
        self.__ros_cle_simulation_factory.initialize()
        self.__ros_cle_simulation_factory.run()
        self.__mocked_rospy.init_node.assert_called_once_with(
            self.__ros_cle_simulation_factory.ROS_CLE_NODE_NAME
        )
        self.__mocked_rospy.Service.assert_any_call(
            self.__ros_cle_simulation_factory.ROS_CLE_URI_PREFIX + "/start_new_simulation",
            srv.StartNewSimulation,
            self.__ros_cle_simulation_factory.start_new_simulation
        )
        self.__mocked_rospy.Service.assert_any_call(
            self.__ros_cle_simulation_factory.ROS_CLE_URI_PREFIX + "/version",
            srv.GetVersion,
            self.__ros_cle_simulation_factory.get_version
        )
        self.assertEqual(self.__mocked_rospy.Service.call_count, 2)
        self.__mocked_rospy.spin.assert_called_once_with()

    @patch('hbp_nrp_cle.cle.ROSCLESimulationFactory.logger')
    @patch('hbp_nrp_cle.cle.ROSCLESimulationFactory.os')
    @patch('hbp_nrp_cle.robotsim.LocalGazebo.os')
    def test_start_new_simulation_dead_thread(self, mocked_os, mocked_cle_os, mocked_logger):
        self.mockThreading()
        self.__ros_cle_simulation_factory.\
            running_simulation_thread.is_alive = MagicMock(return_value=False)

        with patch('hbp_nrp_cle.robotsim.LocalGazebo.LocalGazeboServerInstance.gazebo_master_uri')\
            as mock_gazebo_master_uri:
            mock_gazebo_master_uri.__get__ = Mock(return_value=None)
            messages = self.__ros_cle_simulation_factory.start_new_simulation(
                self.mocked_service_request
            )

        self.assertEqual(mocked_logger.info.call_count, 3)
        self.assertEqual(mocked_logger.error.call_count, 0)

        self.assertEqual(self.__mocked_threading.Thread.call_count, 1)
        self.assertEqual(
            self.__ros_cle_simulation_factory.running_simulation_thread,
            self.__mocked_thread
        )
        self.assertTrue(self.__mocked_thread.daemon)
        self.assertTrue(self.__mocked_thread.start.called)
        self.assertEqual(len(messages), 2)
        result = messages[0]
        error_message = messages[1]

        self.assertTrue(result)
        self.assertEqual(error_message, "")

    @patch('hbp_nrp_cle.cle.ROSCLESimulationFactory.logger')
    def test_start_new_simulation_living_thread(self, mocked_logger):
        self.mockThreading()
        self.__ros_cle_simulation_factory.running_simulation_thread.is_alive = MagicMock(
            return_value=True
        )

        messages = self.__ros_cle_simulation_factory.start_new_simulation(
            self.mocked_service_request
        )

        self.assertEqual(mocked_logger.info.call_count, 1)
        self.assertEqual(mocked_logger.error.call_count, 1)

        self.assertNotEqual(self.__ros_cle_simulation_factory.running_simulation_thread,
                            self.__mocked_thread)
        self.__ros_cle_simulation_factory.running_simulation_thread.is_alive.\
            assert_called_once_with()

        self.assertEqual(len(messages), 2)
        result = messages[0]
        error_message = messages[1]

        self.assertFalse(result)
        self.assertNotEqual(error_message, "")

    @log_capture(level=logging.WARNING)
    def test_set_up_logger(self, logcapture):
        ROSCLESimulationFactory.set_up_logger('cle_logfile.txt')
        self.assertTrue(os.path.isfile('cle_logfile.txt'))
        os.remove('cle_logfile.txt')

        ROSCLESimulationFactory.set_up_logger(None)
        logcapture.check((
            'hbp_nrp_cle',
            'WARNING',
            'Could not write to specified logfile or no logfile specified, logging to stdout now!'
        ))
        self.assertEqual(logging.getLogger().getEffectiveLevel(), logging.INFO)

        ROSCLESimulationFactory.set_up_logger(None, True)
        self.assertEqual(logging.getLogger().getEffectiveLevel(), logging.DEBUG)

    def test_get_version(self):
        cle_version = str(self.__ros_cle_simulation_factory.get_version(None))
        self.assertEqual(cle_version, hbp_nrp_cle.__version__)

if __name__ == '__main__':
    unittest.main()
