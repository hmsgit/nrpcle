"""
Helper class that takes care of making the appropriate ROS call(s) to
control a simulation.
"""

import rospy
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
        self.cle_start = rospy.ServiceProxy('/ros_cle_simulation/start', Empty)
        self.cle_start.wait_for_service()
        self.cle_pause = rospy.ServiceProxy('/ros_cle_simulation/pause', Empty)
        self.cle_pause.wait_for_service()
        self.cle_stop = rospy.ServiceProxy('/ros_cle_simulation/stop', Empty)
        self.cle_stop.wait_for_service()
        self.cle_reset = rospy.ServiceProxy('/ros_cle_simulation/reset', Empty)
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
