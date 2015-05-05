"""
Advertise a ROS service to start new simulations.
"""

import logging
import rospy
import imp
import threading
import os
import argparse
import sys
import netifaces
import hbp_nrp_cle
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboServerInstance, LocalGazeboBridgeInstance
from hbp_nrp_cle.robotsim.LuganoVizClusterGazebo import LuganoVizClusterGazebo

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"

logger = logging.getLogger('hbp_nrp_cle')
# Warning: We do not use __name__  here, since it translates to __main__
# when this file is run directly (such as python ROSCLESimulationFactory.py)


class ROSCLESimulationFactory(object):
    """
    The purpose of this class is to start simulation thread and to
    provide a ROS service for that. Only one simulation can run at a time.
    """

    ROS_CLE_NODE_NAME = "ros_cle_simulation"
    ROS_CLE_URI_PREFIX = "/" + ROS_CLE_NODE_NAME

    def __init__(self):
        """
        Create a CLE simulation factory.
        """
        logger.debug("Creating new CLE server.")
        self.running_simulation_thread = None
        self.__simulator = None  # The robot simulator
        self.__simulator_bridge = None # The bridge from the simulator to websockets

    def initialize(self):
        """
        Initializes the Simulation factory
        """
        rospy.init_node(self.ROS_CLE_NODE_NAME)
        rospy.Service(
            self.ROS_CLE_URI_PREFIX + "/start_new_simulation",
            srv.StartNewSimulation,
            self.start_new_simulation)
        rospy.Service(
            self.ROS_CLE_URI_PREFIX + "/version",
            srv.GetVersion,
            self.get_version)

    @staticmethod
    def run():
        """
        Start the factory and wait indefinitely. (see rospy.spin documentation)
        """
        rospy.spin()

    # service_request is an unused but mandatory argument
    # pylint: disable=unused-argument
    @staticmethod
    def get_version(service_request):
        """
        Handler for the ROS service. Retrieve the CLE version.
        Warning: Multiprocesses can not be used: https://code.ros.org/trac/ros/ticket/972

        :param: service_request: ROS service message (defined in hbp ROS packages)
        """
        return str(hbp_nrp_cle.__version__)

    def start_new_simulation(self, service_request):
        """
        Handler for the ROS service. Spawn a new simulation.
        Warning: Multiprocesses can not be used: https://code.ros.org/trac/ros/ticket/972

        :param: service_request: ROS service message (defined in hbp ROS packages)
        """
        logger.info("Start new simulation request")
        result = False
        error_message = ""

        if ((self.running_simulation_thread is None) or
                (not self.running_simulation_thread.is_alive())):
            logger.info("No simulation running, starting a new simulation.")

            gz_class = None
            if service_request.gzserver_host == 'local':
                gz_class = LocalGazeboServerInstance
            elif service_request.gzserver_host == 'lugano':
                gz_class = LuganoVizClusterGazebo
            else:
                raise Exception("Invalid value for gzserver_host")

            self.__simulator = gz_class()

            local_ip = netifaces.ifaddresses('eth0')[netifaces.AF_INET][0]['addr']
            ros_master_uri = os.environ.get("ROS_MASTER_URI")
            ros_master_uri = ros_master_uri.replace('localhost', local_ip)
            self.__simulator.start(ros_master_uri)
            # Restart rosbridge since this piece of software is quite unstable.
            logger.info("Restarting rosbridge")
            os.system("supervisorctl restart rosbridge")
            self.__simulator_bridge = LocalGazeboBridgeInstance()
            if self.__simulator.gazebo_master_uri is None:
                # Restart the bridge to make sure that we have a new instance.
                self.__simulator_bridge.restart('', '')
            else:
                logger.info(
                    "Starting gzweb with gazebo master sitting at " +
                    self.__simulator.gazebo_master_uri)
                os.environ['GAZEBO_MASTER_URI'] = self.__simulator.gazebo_master_uri
                gzhost, gzport = self.__simulator.gazebo_master_uri.replace(
                    'http://', '').split(':')
                # Restart the bridge to make sure that we have a new instance.
                self.__simulator_bridge.restart(gzhost, gzport)

            # In the future, it would be great to move the CLE script generation logic here.
            # For the time beeing, we rely on the calling process to send us this thing.
            self.running_simulation_thread = threading.Thread(
                target=self.__simulation,
                args=(service_request.environment_file,
                      service_request.generated_cle_script_file)
            )
            self.running_simulation_thread.daemon = True
            logger.info("Spawning new thread that will manage the experiment execution.")
            self.running_simulation_thread.start()
            result = True
        else:
            error_message = """Trying to initialize a new simulation even though the
                previous one has not been terminated."""
            logger.error(error_message)
            result = False
        return [result, error_message]

    def __simulation(self, environment_file, generated_cle_script_file):
        """
        Main simulation method. Start the simulation from the given script file.

        :param: environment_file: Gazebo world file containing
                                  the environment (without the robot)
        :param: generated_cle_script_file: Generated CLE python script (main loop)
        """
        logger.info("Preparting new simulation with environment file: " +
                    environment_file +
                    " and generated script file " +
                    generated_cle_script_file + ".")
        logger.info("Starting the experiment closed loop engine.")
        experiment_generated_script = imp.load_source(
            'experiment_generated_script', generated_cle_script_file)
        experiment_generated_script.cle_function(environment_file)
        logger.info("Stopping robotics simulator")
        self.__simulator.stop()
        self.__simulator = None
        self.__simulator_bridge.stop()
        self.__simulator_bridge = None


def set_up_logger(logfile_name):
    """
    Configure the root logger of the CLE application
    :param: logfile_name: name of the file created to collect logs
    """
    # We initialize the logging in the startup of the whole CLE application.
    # This way we can access the already set up logger in the children modules.
    # Also the following configuration can later be easily stored in an external
    # configuration file (and then set by the user).
    log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'

    try:
        file_handler = logging.FileHandler(logfile_name)
        file_handler.setFormatter(logging.Formatter(log_format))
        logging.root.setLevel(logging.DEBUG)
        logging.root.addHandler(file_handler)
    except (AttributeError, IOError) as _:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(logging.Formatter(log_format))
        logging.root.setLevel(logging.DEBUG)
        logging.root.addHandler(console_handler)
        logger.warn("Could not write to specified logfile or no logfile specified, " +
                    "logging to stdout now!")


if __name__ == '__main__':
    if os.environ["ROS_MASTER_URI"] == "":
        raise Exception("You should run ROS first.")

    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the CLE logfile')
    args = parser.parse_args()
    server = ROSCLESimulationFactory()
    server.initialize()
    set_up_logger(args.logfile)
    server.run()
    logger.info("CLE Server exiting.")
