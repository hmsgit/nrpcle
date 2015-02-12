"""
ROSCLEServer unit test
"""

from hbp_nrp_cle.cle import ROSCLEServer
import logging
from mock import patch, MagicMock
from testfixtures import log_capture
import unittest
import json


__author__ = 'HBP NRP software team'


class TestROSCLEServer(unittest.TestCase):

    LOGGER_NAME = ROSCLEServer.__name__

    def setUp(self):
        unittest.TestCase.setUp(self)

        # Mock the respective objects and make them available for all tests.
        # Also have a look at the following link:
        # https://docs.python.org/3.5/library/unittest.mock-examples.html#applying-the-same-patch-to-every-test-method
        cle_patcher = patch('hbp_nrp_cle.cle.CLEInterface.IClosedLoopControl')
        rospy_patcher = patch('hbp_nrp_cle.cle.ROSCLEServer.rospy')

        # Ensure that the patchers are cleaned up correctly even in exceptional cases
        # e.g. when an exception was thrown.
        self.addCleanup(cle_patcher.stop)
        self.addCleanup(rospy_patcher.stop)

        self.__mocked_cle = cle_patcher.start()
        self.__mocked_rospy = rospy_patcher.start()

        # Set up our object under test and get sure it calls rospy.init in its
        # constructor.
        self.__ros_cle_server = ROSCLEServer.ROSCLEServer()
        self.__mocked_rospy.init_node.assert_called_with('ros_cle_simulation')

        # Be sure we create a publisher exactly once.
        self.assertEqual(1, self.__mocked_rospy.Publisher.call_count)

        # Assure that also the publish method of the rospy.Publisher is
        # injected as a mock here so that we can use it later in our single
        # test methods
        self.__mocked_ros_status_pub = ROSCLEServer.rospy.Publisher()
        self.__mocked_ros_status_pub.publish = MagicMock()
        self.__mocked_rospy.Publisher.return_value = self.__mocked_ros_status_pub

    def tearDown(self):
        # remove all handlers after each test!
        logging.getLogger(self.LOGGER_NAME).handlers = []

    def test_prepare_initialization(self):
        self.__mocked_cle.is_initialized = False
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.assertEqual(4, self.__mocked_rospy.Service.call_count)
        self.assertEqual(1, self.__mocked_cle.initialize.call_count)

    # TODO(Stefan) test the main method as well!
    def test_main(self):
        pass

    def test_run(self):
        self.__ros_cle_server.run()
        self.assertTrue(self.__mocked_rospy.spin.called)
        self.assertEqual(1, self.__mocked_rospy.spin.call_count)

    def test_notify_start_task(self):
        task_name = 'test_name'
        subtask_name = 'test_subtaskname'
        number_of_subtasks = 1
        block_ui = False
        self.__ros_cle_server.notify_start_task(
            task_name, subtask_name, number_of_subtasks, block_ui)
        self.assertEqual(1, self.__mocked_ros_status_pub.publish.call_count)
        message = {'progress': {'task': task_name,
                                'subtask': subtask_name,
                                'number_of_subtasks': number_of_subtasks,
                                'subtask_index': 0,
                                'block_ui': block_ui}}
        self.__mocked_ros_status_pub.publish.assert_called_with(
            json.dumps(message))

    @log_capture(level=logging.WARNING)
    def test_notify_current_task(self, logcapture):
        self.__ros_cle_server.notify_current_task("new_subtask", True, True)
        logcapture.check(
            (self.LOGGER_NAME, 'WARNING', "Can't update a non existing task.")
        )

    @log_capture(level=logging.WARNING)
    def test_notify_finish_task_no_task(self, logcapture):
        self.__ros_cle_server.notify_finish_task()
        logcapture.check(
            (self.LOGGER_NAME, 'WARNING', "Can't finish a non existing task.")
        )


if __name__ == '__main__':
    unittest.main()
