"""
ROS wrapper around the CLE
"""

__author__ = "Lorenzo Vannucci"

import rospy
import threading
from python_cle.cle.CLEInterface import IClosedLoopControl
from std_srvs.srv import Empty


class ROSCLEClient(object):
    """
    A ROS server wrapper around the Closed Loop Engine.
    """

    def __init__(self):
        """
        Create the wrapper client
        """

        # create service proxies
        self.cle_start = rospy.ServiceProxy('/clewrapper/start', Empty)
        self.cle_start.wait_for_service()
        self.cle_pause = rospy.ServiceProxy('/clewrapper/pause', Empty)
        self.cle_pause.wait_for_service()
        self.cle_stop = rospy.ServiceProxy('/clewrapper/stop', Empty)
        self.cle_stop.wait_for_service()
        self.cle_reset = rospy.ServiceProxy('/clewrapper/reset', Empty)
        self.cle_reset.wait_for_service()

    def start(self):
        """
        Start the simulation.
        """
        self.cle_start()

    def pause(self):
        """
        Pause the simulation.
        """
        self.cle_pause()

    def stop(self):
        """
        Stop the simulation.
        """
        self.cle_stop()

    def reset(self):
        """
        Reset the simulation.
        """
        self.cle_reset()


# pylint: disable=R0902
# the attributes are reasonable in this case
class ROSCLEServer(threading.Thread):
    """
    A ROS server wrapper around the Closed Loop Engine.
    """

    def __init__(self, cle):
        """
        Create the wrapper server
        :param cle: the closed loop engine
        """
        super(ROSCLEServer, self).__init__()
        self.daemon = True

        # declare ROS services
        try:
            rospy.init_node("ROSCLEServer")
        except rospy.exceptions.ROSException:
            print "ROS node already initialized"
        self.srv_start = rospy.Service('/clewrapper/start',
                                       Empty, self.start_handler)
        self.srv_pause = rospy.Service('/clewrapper/pause',
                                       Empty, self.pause_handler)
        self.srv_stop = rospy.Service('/clewrapper/stop',
                                      Empty, self.stop_handler)
        self.srv_reset = rospy.Service('/clewrapper/reset',
                                       Empty, self.reset_handler)

        # initialize the simulation if necessary
        assert isinstance(cle, IClosedLoopControl)
        self.cle = cle
        if not self.cle.is_initialized:
            self.cle.initialize()

        # initialize flags
        self.running_flag = threading.Event()
        self.running_flag.clear()
        self.reset = False
        self.stop = False
        self.simulate = False

    def main(self):
        """
        Main control loop.
        """
        self.start()

        while True:
            self.running_flag.wait()
            self.running_flag.clear()

            if self.reset:  # reset simulation
                self.reset = False
                self.cle.reset()

            if self.stop:  # stop simulation and quit
                self.stop = False
                break

            if self.simulate:  # simulate until cle.stop is called
                self.simulate = False
                self.cle.start()

    def start_handler(self, _):
        """
        Handler for both the start and the resume calls.
        """
        self.simulate = True
        self.running_flag.set()
        return []

    def pause_handler(self, _):
        """
        Handler for the pause call.
        """
        self.cle.stop()
        self.cle.wait_step()
        return []

    def stop_handler(self, _):
        """
        Handler for the stop call.
        """
        self.stop = True
        self.running_flag.set()
        self.cle.stop()
        self.cle.wait_step()
        return []

    def reset_handler(self, _):
        """
        Handler for the reset call.
        """
        self.reset = True
        self.running_flag.set()
        self.cle.stop()
        self.cle.wait_step()
        return []

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        rospy.spin()
