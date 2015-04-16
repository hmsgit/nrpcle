"""
This module starts the ROSCLESimulationFactory pushing all exceptions to a dedicated ROS topic
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.cle import ROSCLESimulationFactory
import rospy
import std_msgs.msg
import sys
import logging

error_publisher = None

def unhandled_exception(type, value, traceback):
    error_message = "Unhandled exception of type {0}: {1}".format(type, value)
    if error_publisher is not None:
        error_publisher.publish(error_message)
    print error_message
    print traceback

if __name__ == "__main__":
    sys.excepthook = unhandled_exception

    ROSCLESimulationFactory.set_up_logger(None)

    server = ROSCLESimulationFactory.ROSCLESimulationFactory()
    print "Initialize CLE server"
    server.initialize()
    print "Create publisher for exceptions"
    error_publisher = rospy.Publisher("/integration_test/exceptions", std_msgs.msg.String, queue_size=10)
    print "Starting CLE server"
    server.run()
    print "CLE server shutdown"