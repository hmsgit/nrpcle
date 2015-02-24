"""
ROS wrapper around the CLE
"""
import json
import logging
import threading
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of the CLE (this) repository.
from ROSCLEServicesDefinitions import srv
from hbp_nrp_cle.cle.ROSCLEState import ROSCLEState

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"
logger = logging.getLogger(__name__)


# pylint: disable=R0902
# the attributes are reasonable in this case
class ROSCLEServer(threading.Thread):

    """
    A ROS server wrapper around the Closed Loop Engine.
    """
    ROS_CLE_NODE_NAME = "ros_cle_simulation"
    ROS_CLE_URI_PREFIX = "/" + ROS_CLE_NODE_NAME

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
                               (type(self).__name__))

        def stop_simulation(self):
            raise RuntimeError('You cannot stop the simulation while in %s.' %
                               (type(self).__name__))

        def pause_simulation(self):
            raise RuntimeError('You cannot pause the simulation while in %s.' %
                               (type(self).__name__))

        def start_simulation(self):
            raise RuntimeError('You cannot start the simulation while in %s.' %
                               (type(self).__name__))

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

    def __init__(self):
        """
        Create the wrapper server
        """
        super(ROSCLEServer, self).__init__()
        self.daemon = True

        # ROS allows multiple calls to init_node, as long as
        # the arguments are the same.
        rospy.init_node(self.ROS_CLE_NODE_NAME)

        self.__event_flag = threading.Event()
        self.__event_flag.clear()
        self.__state = ROSCLEServer.InitialState(self)

        self.__service_start = None
        self.__service_pause = None
        self.__service_stop = None
        self.__service_reset = None
        self.__service_state = None
        self.__cle = None

        self.__to_be_executed_within_main_thread = None

        self.__ros_status_pub = rospy.Publisher(
            self.ROS_CLE_URI_PREFIX + '/status', String)

        self.__current_task = None
        self.__current_subtask_count = 0
        self.__current_subtask_index = 0

    def set_state(self, state):
        """
        Sets the current state of the ROSCLEServer. This is used from the State Pattern
        implementation.
        """
        self.__state = state
        self.__event_flag.set()

    def prepare_simulation(self, cle):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.

        :param __cle: the closed loop engine
        """
        self.__cle = cle
        if not self.__cle.is_initialized:
            self.__cle.initialize()

        logger.info("Registering ROS Service handlers")

        # We have to use lambdas here (!) because otherwise we bind to the state which is in place
        # during the time we set the callback! I.e. we would bind directly to the initial state.
        # The x parameter is defined because of the architecture of rospy.
        # rospy is expecting to have handlers which takes two arguments (self and x). The
        # second one holds all the arguments sent through ROS (defined in the srv file).
        # Even when there is no input argument for the service, rospy requires this.

        # pylint: disable=unnecessary-lambda
        self.__service_start = rospy.Service(self.ROS_CLE_URI_PREFIX + '/start', Empty,
                                             lambda x: self.__state.start_simulation())

        self.__service_pause = rospy.Service(self.ROS_CLE_URI_PREFIX + '/pause', Empty,
                                             lambda x: self.__state.pause_simulation())

        self.__service_stop = rospy.Service(self.ROS_CLE_URI_PREFIX + '/stop', Empty,
                                            lambda x: self.__state.stop_simulation())

        self.__service_reset = rospy.Service(self.ROS_CLE_URI_PREFIX + '/reset', Empty,
                                             lambda x: self.__state.reset_simulation())

        self.__service_state = rospy.Service(
                    self.ROS_CLE_URI_PREFIX + '/state',
                    srv.get_simulation_state,
                    lambda x: str(self.__state))

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

            if (self.__to_be_executed_within_main_thread is not None):
                self.__to_be_executed_within_main_thread()
                self.__to_be_executed_within_main_thread = None
            self.__event_flag.wait()  # waits until an event is set
            self.__event_flag.clear()

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
        logger.info("Unregister status topic")
        self.__ros_status_pub.unregister()
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

    # pylint: disable=no-self-use
    def start_simulation(self):
        """
        Handler for the CLE start() call, also used for resuming after pause().
        """
        self.__to_be_executed_within_main_thread = self.__cle.start
        # Next line is needed as a result for a ROS Service call!
        # Returning None (i.e. omitting the return statement) would produce an error.
        return []

    def pause_simulation(self):
        """
        Handler for the CLE pause() call. Actually call to CLE stop(), as CLE has no real pause().
        """
        # CLE has no explicit pause command, use stop() instead
        self.__cle.stop()
        # CLE stop() only sets a flag, so we have to wait until current simulation step is finished
        self.__cle.wait_step()
        # Next line is needed as a result for a ROS Service call!
        # Returning None (i.e. omitting the return statement) would produce an error.
        return []

    def stop_simulation(self):
        """
        Handler for the CLE stop() call, includes waiting for the current simulation step to finish.
        """
        self.__cle.stop()
        # CLE stop() only sets a flag, so we have to wait until current simulation step is finished
        self.__cle.wait_step()
        # Next line is needed as a result for a ROS Service call!
        # Returning None (i.e. omitting the return statement) would produce an error.
        return []

    def reset_simulation(self):
        """
        Handler for the CLE reset() call, additionally triggers a CLE stop().
        """
        # CLE reset() already includes stop() and wait_step()
        self.__to_be_executed_within_main_thread = self.__cle.reset
        # Next line is needed as a result for a ROS Service call!
        # Returning None (i.e. omitting the return statement) would produce an error.
        return []

    def __push_status_on_ros(self, message):
        """
        Push the given message to ROS

        :param: message: The message to publish
        """
        self.__ros_status_pub.publish(message)
