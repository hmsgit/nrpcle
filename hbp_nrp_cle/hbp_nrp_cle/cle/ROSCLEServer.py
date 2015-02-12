"""
ROS wrapper around the CLE
"""
import json
import logging
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import threading

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
        self.__reset = False
        self.__stop = False
        self.__simulate = False

        self.__service_start = None
        self.__service_pause = None
        self.__service_stop = None
        self.__service_reset = None
        self.__cle = None

        self.__ros_status_pub = rospy.Publisher(
            self.ROS_CLE_URI_PREFIX + '/status', String)

        self.__current_task = None
        self.__current_subtask_count = 0
        self.__current_subtask_index = 0

    def prepare_simulation(self, cle):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.
        :param __cle: the closed loop engine
        """
        self.__cle = cle
        if not self.__cle.is_initialized:
            self.__cle.initialize()

        self.__service_start = rospy.Service(self.ROS_CLE_URI_PREFIX + '/start',
                                             Empty, self.__start_handler)
        self.__service_pause = rospy.Service(self.ROS_CLE_URI_PREFIX + '/pause',
                                             Empty, self.__pause_handler)
        self.__service_stop = rospy.Service(self.ROS_CLE_URI_PREFIX + '/stop',
                                            Empty, self.__stop_handler)
        self.__service_reset = rospy.Service(self.ROS_CLE_URI_PREFIX + '/reset',
                                             Empty, self.__reset_handler)

    def main(self):
        """
        Main control loop.
        """
        self.start()

        while True:
            self.__event_flag.wait()  # waits until an event is set
            self.__event_flag.clear()

            if self.__reset:  # reset simulation
                self.__reset = False
                self.__cle.reset()

            if self.__stop:  # stop simulation and quit
                self.__stop = False
                break

            if self.__simulate:  # simulate until __cle.stop is called
                self.__simulate = False
                self.__cle.start()

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        rospy.spin()

    def shutdown(self):
        """
        Unregister every ROS services and topics
        """
        logger.info("Unregister status topic ")
        self.__ros_status_pub.unregister()
        logger.info("shutting down start service ")
        self.__service_start.shutdown()
        logger.info("shutting down pause services ")
        self.__service_pause.shutdown()
        logger.info("shutting down stop services ")
        self.__service_stop.shutdown()
        logger.info("shutting down reset services ")
        self.__service_reset.shutdown()
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

    def __start_handler(self, _):
        """
        Handler for both the start and the resume calls.
        """
        self.__simulate = True
        self.__event_flag.set()
        return []

    def __pause_handler(self, _):
        """
        Handler for the pause call.
        """
        self.__cle.stop()
        self.__cle.wait_step()
        return []

    def __stop_handler(self, _):
        """
        Handler for the __stop call.
        """
        self.__stop = True
        self.__event_flag.set()
        self.__cle.stop()
        self.__cle.wait_step()
        return []

    def __reset_handler(self, _):
        """
        Handler for the __reset call.
        """
        self.__reset = True
        self.__event_flag.set()
        self.__cle.stop()
        self.__cle.wait_step()
        return []

    def __push_status_on_ros(self, message):
        """
        Push the given message to ROS
        :param: message: The message to publish
        """
        self.__ros_status_pub.publish(message)
