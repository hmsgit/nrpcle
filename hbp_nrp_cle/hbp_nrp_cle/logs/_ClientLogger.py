"""
Logger to be used in tranfer functions
"""

import rospy

from std_msgs.msg import String
from hbp_nrp_cle.cle import TOPIC_LOGS


class ClientLogger(object):
    """
    Object that can be used to send log messages to the ROS topic susbscribed by the client
    """
    def __init__(self):
        self.__log_publisher = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)

    def info(self, msg):
        """
        Logs a message os severity level 'Info'
        :param msg: message to log
        """
        self.__log_publisher.publish(msg)
