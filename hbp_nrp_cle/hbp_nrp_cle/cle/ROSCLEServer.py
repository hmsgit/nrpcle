"""
ROS wrapper around the CLE
"""
import json
import logging
import threading
import rospy
import math
import numpy

from std_msgs.msg import String
from std_srvs.srv import Empty
from threading import Thread, Event

import textwrap
import re

from RestrictedPython import compile_restricted

# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from hbp_nrp_cle.common import SimulationFactoryCLEError
from hbp_nrp_cle.cle import ROS_CLE_NODE_NAME, SERVICE_SIM_START_ID, \
    TOPIC_STATUS, TOPIC_TRANSFER_FUNCTION_ERROR, \
    SERVICE_SIM_PAUSE_ID, SERVICE_SIM_STOP_ID, SERVICE_SIM_RESET_ID, SERVICE_SIM_STATE_ID, \
    SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_SET_TRANSFER_FUNCTION, SERVICE_DELETE_TRANSFER_FUNCTION
from hbp_nrp_cle.cle.ROSCLEState import ROSCLEState
from hbp_nrp_cle.cle import ros_handler
import hbp_nrp_cle.tf_framework as tf_framework
from hbp_nrp_cle.tf_framework import TFLoadingException

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"
logger = logging.getLogger(__name__)


# from http://stackoverflow.com/questions/12435211/
#             python-threading-timer-repeat-function-every-n-seconds
class DoubleTimer(Thread):
    """
    Timer that runs two functions, one every n1 seconds and the other
    every n2 seconds, using only one thread
    """

    def __init__(self, interval1, callback1, interval2, callback2):
        """
        Construct the timer. To make it work properly, interval2 must be a
        multiple of interval1.

        :param interval1: the time interval of the first function
        :param callback1: the first function to be called
        :param interval2: the time interval of the second function
        :param callback2: the second function to be called
        """
        Thread.__init__(self)
        self.setDaemon(True)
        if interval1 <= 0 or interval2 <= 0:
            logger.error("interval1 or interval2 must be positive")
            raise ValueError("interval1 or interval2 must be positive")
        if math.fmod(interval2, interval1) > 1e-10:
            logger.error("interval2 of Double timer is not a multiple \
                          of interval1")
            raise ValueError("interval2 is not a multiple of interval1")
        self.interval1 = interval1
        self.callback1 = callback1
        self.interval2 = interval2
        self.callback2 = callback2
        self.stopped = Event()
        self.stopped.clear()
        self.counter = 0
        self.expiring = False

    def run(self):
        """
        Exec the function and restart the timer.
        """
        while not self.stopped.wait(self.interval1):
            self.callback1()
            if self.expiring:
                self.counter += 1
                if self.counter >= self.interval2 / self.interval1:
                    self.counter = 0
                    self.expiring = False
                    self.callback2()

    def is_expiring(self):
        """
        Return True if the second callback is active, False otherwise.
        """
        return self.expiring

    def cancel_all(self):
        """
        Cancel the timer.
        """
        self.disable_second_callback()
        self.stopped.set()

    def enable_second_callback(self):
        """
        Enable calling of the second callback (one call only).
        """
        self.expiring = True

    def disable_second_callback(self):
        """
        Disable calling of the second callback and reset its timer.
        """
        self.expiring = False
        self.counter = 0

    def get_remaining_time(self):
        """
        Get remaining time before the second callback is executed.
        If the callback is disabled, the function will return the full
        interval2.

        :return: the remaining time before the callback
        """
        if not self.expiring:
            return self.interval2
        else:
            return self.interval2 - self.counter * self.interval1


# pylint: disable=R0902
# the attributes are reasonable in this case
class ROSCLEServer(threading.Thread):
    """
    A ROS server wrapper around the Closed Loop Engine.
    """
    STATUS_UPDATE_INTERVAL = 1.0

    class State(object):
        """
        Represents the state in which a ROSCLEServer instance can be.
        This is the base class defining the basic behavior, which means
        that no transitions are to be made to other states and this base
        state itself is not a final state.
        """
        def __init__(self, context):
            self._context = context

        # We disable the docstring here since there is nothing more to say than
        # what the method name already reveals.
        # pylint: disable=missing-docstring
        def reset_simulation(self):
            raise RuntimeError('You cannot reset the simulation while in %s.' %
                               (type(self).__name__, ))

        def stop_simulation(self):
            raise RuntimeError('You cannot stop the simulation while in %s.' %
                               (type(self).__name__, ))

        def pause_simulation(self):
            raise RuntimeError('You cannot pause the simulation while in %s.' %
                               (type(self).__name__, ))

        def start_simulation(self):
            raise RuntimeError('You cannot start the simulation while in %s.' %
                               (type(self).__name__, ))

        # pylint: disable=no-self-use
        def is_final_state(self):
            return False

    class InitialState(State):
        """
        The initial state in which an instance of ROSCLEServer starts its lifecycle.
        """
        def start_simulation(self):
            result = self._context.start_simulation()
            self._context.set_state(ROSCLEServer.RunningState(self._context))
            return result

        def stop_simulation(self):
            result = self._context.stop_simulation()
            self._context.set_state(ROSCLEServer.StoppedState(self._context))
            return result

        def __repr__(self):
            return ROSCLEState.INITIALIZED

    class RunningState(State):
        """
        Represents a running ROSCLEServer.
        """
        def reset_simulation(self):
            result = self._context.reset_simulation()
            self._context.set_state(ROSCLEServer.InitialState(self._context))
            return result

        def stop_simulation(self):
            result = self._context.stop_simulation()
            self._context.set_state(ROSCLEServer.StoppedState(self._context))
            return result

        def pause_simulation(self):
            result = self._context.pause_simulation()
            self._context.set_state(ROSCLEServer.PausedState(self._context))
            return result

        def __repr__(self):
            return ROSCLEState.STARTED

    class StoppedState(State):
        """
        Represents a stopped ROSCLEServer.
        """
        def is_final_state(self):
            return True

        def __repr__(self):
            return ROSCLEState.STOPPED

    class PausedState(State):
        """
        Represents a paused ROSCLEServer.
        """
        def start_simulation(self):
            result = self._context.start_simulation()
            self._context.set_state(ROSCLEServer.RunningState(self._context))
            return result

        def stop_simulation(self):
            result = self._context.stop_simulation()
            self._context.set_state(ROSCLEServer.StoppedState(self._context))
            return result

        def reset_simulation(self):
            result = self._context.reset_simulation()
            self._context.set_state(ROSCLEServer.InitialState(self._context))
            return result

        def __repr__(self):
            return ROSCLEState.PAUSED

    def __init__(self, sim_id):
        """
        Create the wrapper server

        :param sim_id: The simulation id
        """
        super(ROSCLEServer, self).__init__()
        self.daemon = True

        # ROS allows multiple calls to init_node, as long as
        # the arguments are the same.
        rospy.init_node(ROS_CLE_NODE_NAME)

        self.__event_flag = threading.Event()
        self.__event_flag.clear()
        self.__done_flag = threading.Event()
        self.__done_flag.clear()
        self.__state = ROSCLEServer.InitialState(self)

        self.__service_start = None
        self.__service_pause = None
        self.__service_stop = None
        self.__service_reset = None
        self.__service_state = None
        self.__service_get_transfer_functions = None
        self.__service_set_transfer_function = None
        self.__service_delete_transfer_function = None
        self.__cle = None

        self.__to_be_executed_within_main_thread = []

        self.__simulation_id = sim_id
        self.__ros_status_pub = rospy.Publisher(
            TOPIC_STATUS,
            String,
            queue_size=10  # Not expecting more that 10hz
        )
        self.__ros_tf_error_pub = rospy.Publisher(
            TOPIC_TRANSFER_FUNCTION_ERROR,
            SimulationFactoryCLEError,
            queue_size=10  # Not expecting more that 10hz
        )

        self.__current_task = None
        self.__current_subtask_count = 0
        self.__current_subtask_index = 0

        # timeout stuff
        self.__timeout = None
        self.__double_timer = None

    def set_state(self, state):
        """
        Sets the current state of the ROSCLEServer. This is used from the State Pattern
        implementation.
        """
        self.__state = state
        self.__event_flag.set()

    def prepare_simulation(self, cle, timeout=600):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.

        :param __cle: the closed loop engine
        :param timeout: the timeout time of the simulation,
            default is 5 minutes
        """
        self.__cle = cle
        if not self.__cle.is_initialized:
            self.__cle.initialize()

        self.__cle.tfm.publish_error_callback = self.__push_tf_error_on_ros

        logger.info("Registering ROS Service handlers")

        # We have to use lambdas here (!) because otherwise we bind to the state which is in place
        # during the time we set the callback! I.e. we would bind directly to the initial state.
        # The x parameter is defined because of the architecture of rospy.
        # rospy is expecting to have handlers which takes two arguments (self and x). The
        # second one holds all the arguments sent through ROS (defined in the srv file).
        # Even when there is no input argument for the service, rospy requires this.

        # pylint: disable=unnecessary-lambda
        self.__service_start = rospy.Service(
            SERVICE_SIM_START_ID(self.__simulation_id), Empty,
            lambda x: self.__state.start_simulation()
        )

        self.__service_pause = rospy.Service(
            SERVICE_SIM_PAUSE_ID(self.__simulation_id), Empty,
            lambda x: self.__state.pause_simulation()
        )

        self.__service_stop = rospy.Service(
            SERVICE_SIM_STOP_ID(self.__simulation_id), Empty,
            lambda x: self.__state.stop_simulation()
        )

        self.__service_reset = rospy.Service(
            SERVICE_SIM_RESET_ID(self.__simulation_id), Empty,
            lambda x: self.__state.reset_simulation()
        )

        self.__service_state = rospy.Service(
            SERVICE_SIM_STATE_ID(self.__simulation_id), srv.GetSimulationState,
            lambda x: str(self.__state)
        )

        self.__service_get_transfer_functions = rospy.Service(
            SERVICE_GET_TRANSFER_FUNCTIONS(self.__simulation_id), srv.GetTransferFunctions,
            self.__get_transfer_function_sources
        )

        self.__service_set_transfer_function = rospy.Service(
            SERVICE_SET_TRANSFER_FUNCTION(self.__simulation_id), srv.SetTransferFunction,
            self.__set_transfer_function
        )

        self.__service_delete_transfer_function = rospy.Service(
            SERVICE_DELETE_TRANSFER_FUNCTION(self.__simulation_id), srv.DeleteTransferFunction,
            self.__delete_transfer_function
        )

        self.__timeout = timeout
        self.__double_timer = DoubleTimer(
            self.STATUS_UPDATE_INTERVAL,
            self.__publish_state_update,
            self.__timeout,
            self.quit_by_timeout
        )
        self.__double_timer.start()
        self.start_timeout()

    def __get_remaining(self):
        """
        Get the remaining time of the simulation
        """
        return self.__double_timer.get_remaining_time()

    # pylint: disable=unused-argument
    @staticmethod
    def __get_transfer_function_sources(request):
        """
        Return the source code of the transfer functions

        :param request: The mandatory rospy request parameter
        """
        return numpy.asarray([tf.source for tf in tf_framework.get_transfer_functions()])

    def __set_transfer_function(self, request):
        """
        Patch a transfer function

        :param request: The mandatory rospy request parameter
        :return: empty string for a successful compilation in restricted mode
                (executed synchronously),
                 an error message otherwise.
        """

        original_name = request.transfer_function_name
        # Delete synchronously the original if needed
        if (original_name):
            tf_framework.delete_transfer_function(original_name)

        # Update transfer function's source code
        new_source = textwrap.dedent(request.transfer_function_source)

        # Check whether the function has a single definition name
        logger.debug(
            "About to compile transfer function originally named "
            + original_name + "\n"
            + "with the following python code: \n"
            + repr(new_source)
        )
        m = re.findall(r"def\s+(\w+)\s*\(", new_source)
        if (len(m) != 1):
            error_msg = original_name
            if (len(m) == 0):
                error_msg += " has no definition name."
            else:
                error_msg += " has multiple definition names."
            error_msg += " Compilation aborted"
            msg = SimulationFactoryCLEError(
                "Transfer Function",
                "NoOrMultipleNames",
                error_msg,
                original_name)
            self.__push_tf_error_on_ros(msg)
            return msg.message

        # Compile (synchronously) transfer function's new code in restricted mode
        new_name = m[0]
        new_code = None
        try:
            new_code = compile_restricted(new_source, '<string>', 'exec')
        except SyntaxError as e:
            message = "Syntax Error while compiling the updated" \
                + " transfer function named " + new_name \
                + " in restricted mode.\n" \
                + str(e)
            logger.error(message)
            msg = SimulationFactoryCLEError(
                "Transfer Function",
                "Compile",
                str(e),
                new_name,
                e.lineno,
                e.offset,
                e.text,
                e.filename
            )
            self.__push_tf_error_on_ros(msg)
            return message

        # Make sure CLE is stopped. If already stopped, these calls are harmless.
        # (Execution of updated code is asynchronous)
        self.__execute_high_priority_function_within_main_thread_with_cle_stopped(
            lambda s=new_source, c=new_code, n=new_name:
            tf_framework.set_transfer_function(s, c, n)
        )
        return ""

    def __delete_transfer_function(self, request):
        """
        Delete an existing transfer function

        :param request: The mandatory rospy request parameter
        :return: always True as this command is executed asynchronously.
                 ROS forces us to return a value.
        """
        self.__execute_high_priority_function_within_main_thread_with_cle_stopped(
            lambda n=request.transfer_function_name:
            tf_framework.delete_transfer_function(n)
        )
        return True

    def __execute_high_priority_function_within_main_thread_with_cle_stopped(self, function):
        """
        Execute a function within main thread. The function will be placed at the highest priority
        in the list of functions to be executed. The CLE will be stopped prior of executing the
        function (in case it was in a started state).
        :param function: Function to be executed
        """

        self.__cle.stop()
        self.__done_flag.wait()
        self.__done_flag.clear()

        self.__to_be_executed_within_main_thread.insert(
            0,  # Make sure that the function is in the first position of the list
            function
        )
        self.__event_flag.set()

    def start_timeout(self):
        """
        Start the timeout on the current simulation
        """
        self.__double_timer.enable_second_callback()
        logger.info("Simulation will timeout in %f seconds", self.__timeout)

    def stop_timeout(self):
        """
        Stop the timeout
        """
        if self.__double_timer.is_expiring:
            self.__double_timer.disable_second_callback()
            logger.info("Timeout stopped")

    def quit_by_timeout(self):
        """
        Stops the simulation
        """
        self.__state.stop_simulation()
        logger.info("Force quitting the simulation")

    def __publish_state_update(self):
        """
        Publish the state and the remaining timeout
        """
        message = {
            'state': str(self.__state),
            'timeout': self.__get_remaining(),
            'simulationTime': int(self.__cle.simulation_time),
            'realTime': int(self.__cle.real_time),
            'transferFunctionsElapsedTime': self.__cle.tf_elapsed_time(),
            'brainsimElapsedTime': self.__cle.brainsim_elapsed_time(),
            'robotsimElapsedTime': self.__cle.robotsim_elapsed_time()
        }
        logger.info(json.dumps(message))
        self.__push_status_on_ros(json.dumps(message))

    # TODO(Stefan)
    # Probably it would be better to only have a run method and get rid of main.
    # This is the conventional use of Thread and users expect to call Thread.start() which
    # in turn calls the run method.
    # The reason why we have a main method here is that we want to have rospy.spin() called
    # from the run method. This can most probably be made much cleaner by using something like
    # "Thread( ... target=rospy.spin)"
    # But we would have to check out first how this exactly works ...
    def main(self):
        """
        Main control loop. From outside only the main method should be called, which calls
        itself self.start() that triggers run().
        """
        self.start()

        while not self.__state.is_final_state():
            if self.__to_be_executed_within_main_thread:
                for function in self.__to_be_executed_within_main_thread:
                    try:
                        function()
                    except TFLoadingException as e:
                        tf_error = SimulationFactoryCLEError(
                            "Transfer Function",
                            "Loading",
                            e.message,
                            e.tf_name
                        )
                        self.__push_tf_error_on_ros(tf_error)

                self.__to_be_executed_within_main_thread = []
                self.__done_flag.set()
            self.__event_flag.wait()  # waits until an event is set
            self.__event_flag.clear()

        self.__publish_state_update()
        logger.info("Finished main loop")

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        rospy.spin()

    def shutdown(self):
        """
        Unregister every ROS services and topics
        """
        self.__double_timer.join()
        logger.info("Shutting down start service")
        self.__service_start.shutdown()
        logger.info("Shutting down pause service")
        self.__service_pause.shutdown()
        logger.info("Shutting down stop service")
        self.__service_stop.shutdown()
        logger.info("Shutting down reset service")
        self.__service_reset.shutdown()
        logger.info("Shutting down state service")
        self.__service_state.shutdown()
        logger.info("Shutting down get_transfer_functions service")
        self.__service_get_transfer_functions.shutdown()
        logger.info("Shutting down set_transfer_function service")
        self.__service_set_transfer_function.shutdown()
        if self.__current_task is not None:
            self.notify_finish_task()
        logger.info("Unregister status topic")
        self.__ros_status_pub.unregister()
        logger.info("Unregister error/transfer_function topic")
        self.__ros_tf_error_pub.unregister()
        self.__cle.shutdown()

    def notify_start_task(self, task_name, subtask_name, number_of_subtasks, block_ui):
        """
        Sends a status notification that a task starts on the ROS status topic.
        This method will save the task name and the task size in class members so that
        it could be reused in subsequent call to the notify_current_task method.

        :param: task_name: Title of the task (example: initializing experiment).
        :param: subtask_name: Title of the first subtask. Could be empty
                (example: loading Virtual Room).
        :param: number_of_subtasks: Number of expected subsequent calls to
                notify_current_task(_, True, _).
        :param: block_ui: Indicate that the client should block any user interaction.
        """
        if self.__current_task is not None:
            logger.warn(
                "Previous task was not closed properly, closing it now.")
            self.notify_finish_task()
        self.__current_task = task_name
        self.__current_subtask_count = number_of_subtasks
        message = {'progress': {'task': task_name,
                                'subtask': subtask_name,
                                'number_of_subtasks': number_of_subtasks,
                                'subtask_index': self.__current_subtask_index,
                                'block_ui': block_ui}}
        self.__push_status_on_ros(json.dumps(message))

    def notify_current_task(self, new_subtask_name, update_progress, block_ui):
        """
        Sends a status notification that the current task is updated with a new subtask.

        :param: subtask_name: Title of the first subtask. Could be empty
                (example: loading Virtual Room).
        :param: update_progress: Boolean indicating if the index of the current subtask
                should be updated (usually yes).
        :param: block_ui: Indicate that the client should block any user interaction.
        """
        if self.__current_task is None:
            logger.warn("Can't update a non existing task.")
            return
        if update_progress:
            self.__current_subtask_index += 1
        message = {'progress': {'task': self.__current_task,
                                'subtask': new_subtask_name,
                                'number_of_subtasks': self.__current_subtask_count,
                                'subtask_index': self.__current_subtask_index,
                                'block_ui': block_ui}}
        self.__push_status_on_ros(json.dumps(message))

    def notify_finish_task(self):
        """
        Sends a status notification that the current task is finished.
        """
        if self.__current_task is None:
            logger.warn("Can't finish a non existing task.")
            return
        message = {'progress': {'task': self.__current_task,
                                'done': True}}
        self.__push_status_on_ros(json.dumps(message))
        self.__current_subtask_count = 0
        self.__current_subtask_index = 0
        self.__current_task = None

    @ros_handler
    def start_simulation(self):
        """
        Handler for the CLE start() call, also used for resuming after pause().
        """
        self.__to_be_executed_within_main_thread.append(self.__cle.start)

    @ros_handler
    def pause_simulation(self):
        """
        Handler for the CLE pause() call. Actually call to CLE stop(), as CLE has no real pause().
        """
        # CLE has no explicit pause command, use stop() instead
        self.__cle.stop()

    @ros_handler
    def stop_simulation(self):
        """
        Handler for the CLE stop() call, includes waiting for the current simulation step to finish.
        """
        self.stop_timeout()
        self.__double_timer.cancel_all()
        self.__cle.stop()

    @ros_handler
    def reset_simulation(self):
        """
        Handler for the CLE reset() call, additionally triggers a CLE stop().
        """
        # we have to call the stop function here, otherwise the main thread
        # will not stop executing the simulation loop
        self.__cle.stop()
        self.stop_timeout()
        self.__done_flag.wait()
        self.__done_flag.clear()
        # CLE reset() already includes stop() and wait_step()
        self.__to_be_executed_within_main_thread.append(self.__cle.reset)
        self.start_timeout()

    def __push_status_on_ros(self, message):
        """
        Push the given message to ROS

        :param: message: The message to publish
        """
        self.__ros_status_pub.publish(message)

    def __push_tf_error_on_ros(self, tf_error):
        """
        Push the given error message message to ROS

        :param: message: The message to publish
        """
        self.__ros_tf_error_pub.publish(tf_error)
