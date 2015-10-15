"""
ROSCLEServer unit test
"""

from hbp_nrp_cle.cle import ROSCLEServer
from hbp_nrp_cle.cle.ROSCLEState import ROSCLEState
import logging
from mock import patch, MagicMock, Mock
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

    def craft_ros_cle_server(self, mock_timeout=False):
        # Set up our object under test and get sure it calls rospy.init in its
        # constructor.
        self.__ros_cle_server = ROSCLEServer.ROSCLEServer(0)
        if mock_timeout:
            self.__ros_cle_server.start_timeout = MagicMock()
            self.__ros_cle_server.stop_timeout = MagicMock()
        self.__ros_cle_server._ROSCLEServer__done_flag = Mock()

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
        self.__mocked_cle.simulation_time = 0
        self.__mocked_cle.real_time = 0
        self.__mocked_cle.tf_elapsed_time = Mock(return_value=0)
        self.__mocked_cle.brainsim_elapsed_time = Mock(return_value=0)
        self.__mocked_cle.robotsim_elapsed_time = Mock(return_value=0)
        self.__mocked_rospy = rospy_patcher.start()

        self.craft_ros_cle_server(True)
        self.__mocked_rospy.init_node.assert_called_with('ros_cle_simulation')

        # Be sure we create as many publishers as required.
        from hbp_nrp_cle.cle import *
        number_of_publishers = 0
        for var_name in vars().keys():
            if (var_name.startswith("TOPIC_")):
                number_of_publishers += 1

        self.assertEqual(number_of_publishers, self.__mocked_rospy.Publisher.call_count)

        # Assure that also the publish method of the rospy.Publisher is
        # injected as a mock here so that we can use it later in our single
        # test methods
        self.__mocked_ros_status_pub = ROSCLEServer.rospy.Publisher()
        self.__mocked_ros_status_pub.publish = MagicMock()
        self.__mocked_rospy.Publisher.return_value = self.__mocked_ros_status_pub

    def tearDown(self):
        # remove all handlers after each test!
        logging.getLogger(self.LOGGER_NAME).handlers = []

    def test_set_state(self):
        self.craft_ros_cle_server()
        st = ROSCLEServer.ROSCLEServer.InitialState(self.__ros_cle_server)
        self.__ros_cle_server.set_state(st)
        self.assertEquals(self.__ros_cle_server._ROSCLEServer__state, st)

    @patch('hbp_nrp_cle.cle.ROSCLEServer.DoubleTimer.enable_second_callback')
    def test_start_timeout(self, mock_enable):
        self.craft_ros_cle_server()
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.__ros_cle_server.start_timeout()
        self.assertGreaterEqual(mock_enable.call_count, 1)

    @patch('hbp_nrp_cle.cle.ROSCLEServer.DoubleTimer')
    def test_stop_timeout(self, mock_double_timer):
        self.craft_ros_cle_server()
        mock_double_timer.expiring = True
        mock_double_timer.disable_second_callback = Mock()
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.__ros_cle_server._ROSCLEServer__double_timer = mock_double_timer
        self.__ros_cle_server.stop_timeout()
        self.assertGreaterEqual(mock_double_timer.disable_second_callback.call_count, 1)

    @patch('hbp_nrp_cle.cle.ROSCLEServer.ROSCLEServer.State')
    def test_quit_by_timeout(self, mock_state):
        self.craft_ros_cle_server()
        mock_state.stop_simulation = Mock()
        self.__ros_cle_server.set_state(mock_state)
        self.__ros_cle_server.quit_by_timeout()
        self.assertEquals(mock_state.stop_simulation.call_count, 1)

    def test_prepare_initialization(self):
        self.__mocked_cle.is_initialized = False
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.assertEqual(8, self.__mocked_rospy.Service.call_count)
        self.assertEqual(1, self.__mocked_cle.initialize.call_count)
        self.assertEqual(1, self.__ros_cle_server.start_timeout.call_count)

    def test_reset_timeout(self):
        self.__mocked_cle.is_initialized = False
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.__ros_cle_server.start_simulation()
        self.__ros_cle_server.pause_simulation()
        self.__ros_cle_server.reset_simulation()
        # start_timeout has been called in prepare_simulation AND in reset_simulation
        self.assertEqual(2, self.__ros_cle_server.start_timeout.call_count)
        self.assertEqual(1, self.__ros_cle_server.stop_timeout.call_count)

    def __get_handlers_for_testing_main(self):
        self.__mocked_cle.is_initialized = True
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)

        # Get the ROS Service handlers; this will always be the third argument.
        arguments = self.__mocked_rospy.Service.call_args_list
        start_handler = arguments[0][0][2]
        pause_handler = arguments[1][0][2]
        stop_handler = arguments[2][0][2]
        reset_handler = arguments[3][0][2]
        state_handler = arguments[4][0][2]
        get_transfer_functions_handler = arguments[5][0][2]
        set_transfer_function_handler = arguments[6][0][2]
        delete_transfer_function_handler = arguments[7][0][2]

        return start_handler, pause_handler, stop_handler, reset_handler, state_handler,\
               get_transfer_functions_handler, set_transfer_function_handler, \
               delete_transfer_function_handler

    @timeout(10, "Main loop did not terminate")
    def test_main_termination(self):
        self.craft_ros_cle_server(True)
        (start_handler, _, stop_handler, _, state_handler, _, _, _) = self.__get_handlers_for_testing_main()
        start_handler(_)
        # start a timer which calls the registered stop handler after 5 seconds
        timer = threading.Timer(5, stop_handler, ['irrelevant_argument'])
        timer.start()
        # now start the "infinite loop"
        self.assertEqual(str(state_handler(_)), ROSCLEState.STARTED)
        self.__ros_cle_server.main()
        self.assertEqual(str(state_handler(_)), ROSCLEState.STOPPED)
        self.__mocked_cle.stop.assert_called_once_with()

    @patch('hbp_nrp_cle.cle.ROSCLEServer.tf_framework')
    def test_get_transfer_functions(self, mocked_tf_framework):
        self.craft_ros_cle_server(True)
        tf = [MagicMock(), MagicMock()]
        tf[0].source = "tf_0 python code"
        tf[1].source = "tf_1 python code"
        mocked_tf_framework.get_transfer_functions = MagicMock(return_value=tf)
        (_, _, _, _, _, get_transfer_functions_handler, _, _) = self.__get_handlers_for_testing_main()
        transfer_functions_from_service = get_transfer_functions_handler(_)
        self.assertEqual(2, len(transfer_functions_from_service))
        self.assertEqual("tf_0 python code", transfer_functions_from_service[0])
        self.assertEqual("tf_1 python code", transfer_functions_from_service[1])

    @patch('hbp_nrp_cle.cle.ROSCLEServer.tf_framework')
    def test_set_transfer_function(self, mocked_tf_framework):
        self.craft_ros_cle_server(True)
        mocked_tf_framework.set_transfer_function = MagicMock(return_value=None)
        mocked_tf_framework.delete_transfer_function = MagicMock()
        (_, _, _, _, _, _, set_transfer_function_handler, _) = self.__get_handlers_for_testing_main()
        request = MagicMock()
        request.transfer_function_name = "tf_0"
        request.transfer_function_source = "def tf0(): \n return 0"
        response = set_transfer_function_handler(request)
        mocked_tf_framework.delete_transfer_function.assert_called_once_with("tf_0")
        self.assertEqual("", response)

        mocked_tf_framework.delete_transfer_function.reset_mock()
        request.transfer_function_name = ""
        request.transfer_function_source = "def tf0(): \n return 0"
        response = set_transfer_function_handler(request)
        self.assertEqual(0, mocked_tf_framework.delete_transfer_function.call_count)
        self.assertEqual("", response)

        request.transfer_function_name = "tf_1"
        request.transfer_function_source = "def (): \n return -1"
        response = set_transfer_function_handler(request)
        self.assertIn("no definition name", response)

        request.transfer_function_name = "tf_2"
        request.transfer_function_source = "def tf_2(): \n return -1 undefined"
        response = set_transfer_function_handler(request)
        self.assertIn("invalid syntax", response)
        self.assertIn("line 2", response)

        request.transfer_function_name = "tf_3"
        request.transfer_function_source = "def tf_3_a(): \n\treturn -1\ndef tf_3_b(): \n\treturn -2"
        response = set_transfer_function_handler(request)
        self.assertIn("multiple definition names", response)

    @patch('hbp_nrp_cle.cle.ROSCLEServer.tf_framework')
    def test_delete_transfer_function(self, mocked_tf_framework):
        self.craft_ros_cle_server(True)
        mocked_tf_framework.delete_transfer_function = MagicMock(return_value=True)
        (_, _, _, _, _, _, _, delete_transfer_function_handler) = self.__get_handlers_for_testing_main()
        request = MagicMock()
        request.transfer_function_name = "tf_0"
        response = delete_transfer_function_handler(request)
        self.assertEqual(True, response)

    def test_run(self):
        self.craft_ros_cle_server(True)
        self.__ros_cle_server.run()
        self.assertTrue(self.__mocked_rospy.spin.called)
        self.assertEqual(1, self.__mocked_rospy.spin.call_count)

    def test_notify_start_task(self):
        self.craft_ros_cle_server(True)
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

    def test_plain_state(self):
        self.craft_ros_cle_server(True)
        # Testing class State
        state = ROSCLEServer.ROSCLEServer.State(None)
        self.assertRaises(RuntimeError, state.start_simulation)
        self.assertRaises(RuntimeError, state.pause_simulation)
        self.assertRaises(RuntimeError, state.stop_simulation)
        self.assertRaises(RuntimeError, state.reset_simulation)
        self.assertFalse(state.is_final_state())

    def test_initialized_state(self):
        self.craft_ros_cle_server(True)
        # Testing class InitialState
        ctx = Mock()
        ctx.start_simulation = Mock()
        ctx.stop_simulation = Mock()

        initialized = ROSCLEServer.ROSCLEServer.InitialState(ctx)
        self.assertRaises(RuntimeError, initialized.pause_simulation)
        self.assertRaises(RuntimeError, initialized.reset_simulation)
        self.assertFalse(initialized.is_final_state())

        initialized.start_simulation()
        initialized.stop_simulation()
        self.assertEqual(ctx.start_simulation.call_count, 1)
        self.assertEqual(ctx.stop_simulation.call_count, 1)
        self.assertEqual(str(initialized), ROSCLEState.INITIALIZED)

    def test_running_state(self):
        self.craft_ros_cle_server(True)
        # Testing class RunningState
        ctx = Mock()
        ctx.start_simulation = Mock()
        ctx.stop_simulation = Mock()
        ctx.pause_simulation = Mock()

        running = ROSCLEServer.ROSCLEServer.RunningState(ctx)
        self.assertRaises(RuntimeError, running.start_simulation)
        self.assertFalse(running.is_final_state())

        running.reset_simulation()
        running.stop_simulation()
        running.pause_simulation()
        self.assertEqual(ctx.reset_simulation.call_count, 1)
        self.assertEqual(ctx.stop_simulation.call_count, 1)
        self.assertEqual(ctx.pause_simulation.call_count, 1)
        self.assertEqual(str(running), ROSCLEState.STARTED)

    def test_paused_state(self):
        self.craft_ros_cle_server(True)
        # Testing class PausedState
        ctx = Mock()
        ctx.start_simulation = Mock()
        ctx.stop_simulation = Mock()
        ctx.reset_simulation = Mock()

        paused = ROSCLEServer.ROSCLEServer.PausedState(ctx)
        self.assertRaises(RuntimeError, paused.pause_simulation)
        self.assertFalse(paused.is_final_state())

        paused.start_simulation()
        paused.stop_simulation()
        paused.reset_simulation()
        self.assertEqual(ctx.start_simulation.call_count, 1)
        self.assertEqual(ctx.stop_simulation.call_count, 1)
        self.assertEqual(ctx.reset_simulation.call_count, 1)
        self.assertEqual(str(paused), ROSCLEState.PAUSED)

    def test_stopped_state(self):
        self.craft_ros_cle_server(True)
        # Testing class StoppedState
        stopped = ROSCLEServer.ROSCLEServer.StoppedState(None)
        self.assertRaises(RuntimeError, stopped.start_simulation)
        self.assertRaises(RuntimeError, stopped.pause_simulation)
        self.assertRaises(RuntimeError, stopped.stop_simulation)
        self.assertRaises(RuntimeError, stopped.reset_simulation)
        self.assertTrue(stopped.is_final_state())
        self.assertEqual(str(stopped), ROSCLEState.STOPPED)

    @patch('hbp_nrp_cle.cle.ROSCLEServer.ROSCLEServer._ROSCLEServer__push_status_on_ros')
    def test_task(self, mock_publisher):
        self.craft_ros_cle_server(True)
        self.__ros_cle_server.notify_start_task('task', 'subtask', 1, False)
        self.assertEquals(mock_publisher.call_count, 1)
        self.__ros_cle_server.notify_current_task('new_subtask', True, False)
        self.assertEquals(mock_publisher.call_count, 2)
        self.__ros_cle_server.notify_finish_task()
        self.assertEquals(mock_publisher.call_count, 3)

    def test_shutdown(self):
        self.craft_ros_cle_server()
        a = self.__ros_cle_server._ROSCLEServer__double_timer = MagicMock()
        b = self.__ros_cle_server._ROSCLEServer__service_start = MagicMock()
        c = self.__ros_cle_server._ROSCLEServer__service_pause = MagicMock()
        d = self.__ros_cle_server._ROSCLEServer__service_stop = MagicMock()
        e = self.__ros_cle_server._ROSCLEServer__service_reset = MagicMock()
        f = self.__ros_cle_server._ROSCLEServer__service_state = MagicMock()
        g = self.__ros_cle_server._ROSCLEServer__current_task = None
        h = self.__ros_cle_server._ROSCLEServer__ros_status_pub = MagicMock()
        i = self.__ros_cle_server._ROSCLEServer__cle = MagicMock()
        j = self.__ros_cle_server._ROSCLEServer__service_get_transfer_functions = MagicMock()
        k = self.__ros_cle_server._ROSCLEServer__service_set_transfer_function = MagicMock()

        self.__ros_cle_server.shutdown()
        for x in [b, c, d, e, f, i]:
            self.assertEquals(x.shutdown.call_count, 1)
        self.assertEquals(a.join.call_count, 1)
        self.assertEquals(h.unregister.call_count, 1)

    @log_capture(level=logging.WARNING)
    def test_notify_current_task(self, logcapture):
        self.craft_ros_cle_server(True)
        self.__ros_cle_server.notify_current_task("new_subtask", True, True)
        logcapture.check(
            (self.LOGGER_NAME, 'WARNING', "Can't update a non existing task.")
        )

    @log_capture(level=logging.WARNING)
    def test_notify_finish_task_no_task(self, logcapture):
        self.craft_ros_cle_server(True)
        self.__ros_cle_server.notify_finish_task()
        logcapture.check(
            (self.LOGGER_NAME, 'WARNING', "Can't finish a non existing task.")
        )


if __name__ == '__main__':
    unittest.main()
