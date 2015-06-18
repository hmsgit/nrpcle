"""
This module contains the script to run the integration test
"""

__author__ = 'GeorgHinkel'

import rospy
import std_msgs.msg
from cle_ros_msgs.msg import SpikeRate
from hbp_nrp_cle.cle.ROSCLESimulationFactoryClient import ROSCLESimulationFactoryClient
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClient
from hbp_nrp_cle.bibi_config.bibi_configuration_script import generate_cle
from threading import Thread
import os
import time
import logging
import subprocess
import sys

failed = False
monitor_called = False

log_format = '%(asctime)s [CLIENT:%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
logger = logging.getLogger(__name__)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(logging.Formatter(log_format))
logger.setLevel(logging.DEBUG)


def unhandled_exception(type, value, traceback):
    global failed
    failed = True
    logger.error("Unhandled exception of type {0}: {1}".format(type, value))
    logger.debug(traceback)

sys.excepthook = unhandled_exception


def monitor_callback(spike_rate):
    global monitor_called
    monitor_called = True
    assert isinstance(spike_rate, SpikeRate)
    if spike_rate.monitorName == "left_wheel_neuron_monitor":
        if not 5 < spike_rate.rate < 30:
            log_message = "Spike rate should have been between 5 and 30" \
                          " but was {0} at simulation time {1}"
            log_message = log_message.format(spike_rate.rate, spike_rate.simulationTime)
            logger.error(log_message)
            global failed
            failed = True

def exception_callback(exc):
    global failed
    failed = True
    logger.exception(exc)


def run_integration_test():
    sim_factory = None

    current_dir = os.path.split(__file__)[0]
    generated_bibi_path = os.path.join(current_dir, '__generated_bibi.py')
    run_masked_simulation = os.path.join(current_dir, 'module_test_simulation_factory.py')

    try:
        # First, start simulation factory
        sim_factory = subprocess.Popen([sys.executable, run_masked_simulation])

        rospy.init_node('module_test_monitor')
        # rospy overrides logging!
        logging.root.addHandler(console_handler)
        rospy.Subscriber("/monitor/population_rate", SpikeRate, monitor_callback)
        rospy.Subscriber("/module_test/exceptions", std_msgs.msg.String, exception_callback)
        monitor_thread = Thread(target=rospy.spin)

        monitor_thread.start()

        logging.info("Generating CLE")
        generate_cle(os.path.join(current_dir, 'milestone2.xml'),
                     generated_bibi_path, 50, 'local', 0)

        logging.info("Creating CLE Server")
        client = ROSCLESimulationFactoryClient()
        client.start_new_simulation('virtual_room/virtual_room.sdf', generated_bibi_path)

        logging.info("Creating CLE Client")
        cle = ROSCLEClient(0)

        logging.info("Starting CLE Server")
        cle.start()
        time.sleep(10)
        logging.info("Stopping CLE Server")
        cle.stop()
        logging.info("Module Test completed, running assertions")

    except Exception, e:
        global failed
        failed = True
        logger.error(repr(e))
        print repr(e)
    finally:
        if sim_factory is not None:
            sim_factory.kill()
        rospy.signal_shutdown("Module test is done")

        if os.path.isfile(generated_bibi_path):
            os.remove(generated_bibi_path)


    if failed or not monitor_called:
        if failed:
            logger.error("An error occurred. Marking the module test to fail.")
        if not monitor_called:
            logger.error("The monitoring callback has not been called. "
                         "Marking module test as failed.")
        exit(1)
    else:
        exit(0)

if __name__ == "__main__":
    run_integration_test()