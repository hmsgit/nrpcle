"""
ROSCLEServer unit test
"""

from hbp_nrp_cle.cle import ROSCLEServer
from hbp_nrp_cle.cle.ROSCLEState import ROSCLEState
import logging
from mock import patch, MagicMock
from testfixtures import log_capture
import unittest
import json
import threading
from functools import wraps
from multiprocessing import Process


__author__ = 'HBP NRP software team'

# Code for timeout decorator from
# http://stackoverflow.com/questions/14366761/
class TimeoutError(Exception):
    pass

def timeout(seconds=5, error_message="Timeout"):
    def decorator(func):
        def wrapper(*args, **kwargs):
            process = Process(None, func, None, args, kwargs)
            process.start()
            process.join(seconds)
            if process.is_alive():
                process.terminate()
                raise TimeoutError(error_message)

        return wraps(func)(wrapper)
    return decorator

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
        self.assertEqual(5, self.__mocked_rospy.Service.call_count)
        self.assertEqual(1, self.__mocked_cle.initialize.call_count)

    def __get_handlers_for_testing_main(self):
        self.__mocked_cle.is_initialized = True
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)

        # Get the ROS Service handlers; this will always be the third argument.
        start_handler = self.__mocked_rospy.Service.call_args_list[0][0][2]
        pause_handler = self.__mocked_rospy.Service.call_args_list[1][0][2]
        stop_handler = self.__mocked_rospy.Service.call_args_list[2][0][2]
        reset_handler = self.__mocked_rospy.Service.call_args_list[3][0][2]
        state_handler = self.__mocked_rospy.Service.call_args_list[4][0][2]

        return (start_handler, pause_handler, stop_handler, reset_handler, state_handler)

    @timeout(10, "Main loop did not terminate")
    def test_main_termination(self):
        (start_handler, _, stop_handler, _, state_handler) = self.__get_handlers_for_testing_main()
        start_handler(_)
        # start a timer which calls the registered stop handler after 5 seconds
        timer = threading.Timer(5, stop_handler, ['irrelevant_argument'])
        timer.start()
        # now start the "infinite loop"
        self.assertEqual(str(state_handler(_)), ROSCLEState.STARTED)
        self.__ros_cle_server.main()
        self.assertEqual(str(state_handler(_)), ROSCLEState.STOPPED)
        self.__mocked_cle.stop.assert_called_once_with()
        self.__mocked_cle.wait_step.assert_called_once_with()

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
