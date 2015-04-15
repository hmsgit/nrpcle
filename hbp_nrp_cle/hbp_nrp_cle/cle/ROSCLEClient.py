"""
Takes care of making the appropriate ROS call(s) to control a simulation.
On the other side of ROS, the calls are handled by ROSCLEServer.py
"""

import logging
import rospy
from std_srvs.srv import Empty
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of the CLE (this) repository.
from cle_ros_msgs import srv
from hbp_nrp_cle.cle.ROSCLEState import ROSCLEState

__author__ = "Lorenzo Vannucci, Daniel Peppicelli"
logger = logging.getLogger(__name__)


class ROSCLEClientException(Exception):
    """
    Exception within the CLE client
    """
    pass


class ROSCLEClient(object):
    """
    Client around the ROS controlled Closed Loop Engine.
    """
    ROS_SERVICE_TIMEOUT = 120
    ROS_CLE_NODE_NAME = "ros_cle_simulation"
    ROS_CLE_URI_PREFIX = "/" + ROS_CLE_NODE_NAME

    def __init__(self):
        """
        Create the wrapper client
        """

        self.__valid = True
        self.__invalid_reason = ""

        # Creates service proxies
        self.__cle_start = self.__init_ros_service(self.ROS_CLE_URI_PREFIX + '/start', Empty)
        self.__cle_pause = self.__init_ros_service(self.ROS_CLE_URI_PREFIX + '/pause', Empty)
        self.__cle_stop = self.__init_ros_service(self.ROS_CLE_URI_PREFIX + '/stop', Empty)
        self.__cle_reset = self.__init_ros_service(self.ROS_CLE_URI_PREFIX + '/reset', Empty)
        self.__cle_state = self.__init_ros_service(self.ROS_CLE_URI_PREFIX + '/state',
                                                   srv.GetSimulationState)

    def __init_ros_service(self, service_name, service_class):
        """
        Initialize a ROS service proxy

        @param: service_name: The name of the service
        @param: service_class: The class (.srv) of the service. This class
                               is generated from the ROS catckin package .srv files.
        """
        handler = None
        if self.__valid:
            try:
                logger.info("Connecting to ROS service " + service_name)
                handler = rospy.ServiceProxy(service_name, service_class)
                handler.wait_for_service(timeout=self.ROS_SERVICE_TIMEOUT)
            except rospy.ROSException:
                # According to the documentation, only a timeout will raise a generic
                # 'ROSException'.
                # http://docs.ros.org/api/rospy/html/rospy-module.html#wait_for_service
                error = "Timeout while connecting to the CLE (waiting on " + service_name + ")."
                logger.error(error)
                raise ROSCLEClientException(error)
        return handler

    def __call_service(self, service):
        """
        Generic call handler for all the ROS service proxies that are based on the Empty class.
        The call handler will mask excpetions from ROS and will discard the client if there is
        something wrong going on.

        @param: handler: The ROS proxy service to call
        """
        if self.__valid:
            try:
                service()
            except rospy.ServiceException:
                self.__valid = False
                self.__invalid_reason = "a previous communication error"
                error_message = "Impossible to communicate with the CLE, discarding the client."
                raise ROSCLEClientException(error_message)
        else:
            raise ROSCLEClientException("Client has been discarded due to " +
                                        self.__invalid_reason +
                                        ".")

    def start(self):
        """
        Start the simulation.
        """
        self.__call_service(self.__cle_start)

    def pause(self):
        """
        Pause the simulation.
        """
        self.__call_service(self.__cle_pause)

    def stop(self):
        """
        Stop the simulation.
        """
        self.__call_service(self.__cle_stop)
        # Once the stop handler has been called, we do not expect it to answer anymore
        self.__valid = False
        self.__invalid_reason = "a previous stop request (triggering automatic disconnection)"

    def reset(self):
        """
        Reset the simulation.
        """
        self.__call_service(self.__cle_reset)

    def get_simulation_state(self):
        """
        Get the simulation state.
        """
        # By default, we assume an experiment is in the stop state
        # (whether it is really stop or if something bad did happen.)
        state = ROSCLEState.STOPPED
        if (self.__valid):
            try:
                state = str(self.__cle_state().state)
            except rospy.ServiceException as e:
                logger.error("Error while trying to retrieve simulation state: " +
                             str(e) +
                             ". Returning stopped as state.")
        else:
            logger.warn("Trying to retrieve the state of a simulation from an invalid client " +
                        "(invalid due to " +
                        self.__invalid_reason +
                        ") ")
        return state
