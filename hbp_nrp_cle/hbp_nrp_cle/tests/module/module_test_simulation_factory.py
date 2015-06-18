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

logger = logging.getLogger("SimFactory")
log_format = '%(asctime)s [SERVER:%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(logging.Formatter(log_format))
logger.setLevel(logging.DEBUG)
logger.addHandler(console_handler)

def unhandled_exception(type, value, traceback):
    error_message = "Unhandled exception of type {0}: {1}".format(type, value)
    if error_publisher is not None:
        error_publisher.publish(error_message)
    print error_message
    print traceback

if __name__ == "__main__":
    sys.excepthook = unhandled_exception

    server = ROSCLESimulationFactory.ROSCLESimulationFactory()
    logger.info("Initialize CLE server")
    server.initialize()
    logger.info("Create publisher for exceptions")
    error_publisher = rospy.Publisher("/module_test/exceptions", std_msgs.msg.String, queue_size=10)
    logger.info("Starting CLE server")
    server.run()
    logger.info("CLE server shutdown")