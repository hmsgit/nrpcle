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
import hbp_nrp_cle
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from hbp_nrp_cle.cle import ROS_CLE_NODE_NAME, SERVICE_START_NEW_SIMULATION, SERVICE_VERSION

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"

logger = logging.getLogger('hbp_nrp_cle')
# Warning: We do not use __name__  here, since it translates to __main__
# when this file is run directly (such as python ROSCLESimulationFactory.py)


class ROSCLESimulationFactory(object):
    """
    The purpose of this class is to start simulation thread and to
    provide a ROS service for that. Only one simulation can run at a time.
    """

    def __init__(self):
        """
        Create a CLE simulation factory.
        """
        logger.debug("Creating new CLE server.")
        self.running_simulation_thread = None
        self.simulation_initialized_event = threading.Event()
        self.simulation_exception_during_init = None

    def initialize(self):
        """
        Initializes the Simulation factory
        """
        rospy.init_node(ROS_CLE_NODE_NAME)
        rospy.Service(
            SERVICE_START_NEW_SIMULATION, srv.StartNewSimulation, self.start_new_simulation
        )
        rospy.Service(SERVICE_VERSION, srv.GetVersion, self.get_version)

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
        error_message = ""
        result = True

        if (self.running_simulation_thread is None) or\
                (not self.running_simulation_thread.is_alive()):
            logger.info("No simulation running, starting a new simulation.")

            # In the future, it would be great to move the CLE script generation logic here.
            # For the time beeing, we rely on the calling process to send us this thing.
            self.simulation_initialized_event.clear()
            self.running_simulation_thread = threading.Thread(
                target=self.__simulation,
                args=(service_request.environment_file,
                      service_request.generated_cle_script_file)
            )
            self.running_simulation_thread.daemon = True
            logger.info("Spawning new thread that will manage the experiment execution.")
            self.running_simulation_thread.start()
            self.simulation_initialized_event.wait()
            # Raising the exception here will send it back through ROS to the ExDBackend.
            if self.simulation_exception_during_init:
                # Known pylint bug: goo.gl/WNg0TJ
                # pylint: disable=raising-bad-type
                raise self.simulation_exception_during_init
        else:
            error_message = "Trying to initialize a new simulation even though the " \
                            "previous one has not been terminated."
            logger.error(error_message)
            result = False

        logger.debug("Start_new_simulation return with status: " + str(result))
        return [result, error_message]

    # pylint: disable=no-self-use
    def __simulation(self, environment_file, generated_cle_script_file):
        """
        Main simulation method. Start the simulation from the given script file.

        :param: environment_file: Gazebo world file containing
                                  the environment (without the robot)
        :param: generated_cle_script_file: Generated CLE python script (main loop)
        """
        logger.info(
            "Preparing new simulation with environment file: %s "
            "and generated script file %s.",
            environment_file, generated_cle_script_file
        )
        logger.info("Starting the experiment closed loop engine.")
        cle_server = models_path = gzweb = gzserver = None
        self.simulation_exception_during_init = None
        experiment_generated_script = imp.load_source(
            'experiment_generated_script', generated_cle_script_file)
        logger.info("Executing script: " + generated_cle_script_file)
        # We want any exception raised during initialization in this tread
        # to be pass to the main thread so that it can be handled properly.
        try:
            [cle_server, models_path, gzweb, gzserver] = experiment_generated_script.\
                cle_function_init(environment_file)
        # pylint: disable=broad-except
        except Exception as e:
            logger.exception("Initialization failed")
            self.simulation_exception_during_init = e
            self.simulation_initialized_event.set()
            return

        logger.info("Initialization done")
        self.simulation_initialized_event.set()

        cle_server.main()
        experiment_generated_script.shutdown(cle_server, models_path, gzweb, gzserver)


def set_up_logger(logfile_name, verbose=False):
    """
    Configure the root logger of the CLE application
    :param: logfile_name: name of the file created to collect logs
    :param: verbose: increase logging verbosity
    """
    # We initialize the logging in the startup of the whole CLE application.
    # This way we can access the already set up logger in the children modules.
    # Also the following configuration can later be easily stored in an external
    # configuration file (and then set by the user).
    log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'

    try:
        file_handler = logging.FileHandler(logfile_name)
        file_handler.setFormatter(logging.Formatter(log_format))
        logging.root.addHandler(file_handler)
    except (AttributeError, IOError) as _:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(logging.Formatter(log_format))
        logging.root.addHandler(console_handler)
        logger.warn("Could not write to specified logfile or no logfile specified, "
                    "logging to stdout now!")
    logging.root.setLevel(logging.DEBUG if verbose else logging.INFO)

if __name__ == '__main__':
    if os.environ["ROS_MASTER_URI"] == "":
        raise Exception("You should run ROS first.")

    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the CLE logfile')
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    parser.add_argument('-p', '--pycharm',
                        dest='pycharm',
                        help='debug with pyCharm. IP adress and port are needed.',
                        nargs='+')
    args = parser.parse_args()

    if (args.pycharm):
        # pylint: disable=import-error
        import pydevd
        pydevd.settrace(args.pycharm[0],
                        port=int(args.pycharm[1]),
                        stdoutToServer=True,
                        stderrToServer=True)

    server = ROSCLESimulationFactory()
    server.initialize()
    set_up_logger(args.logfile, args.verbose)
    server.run()
    logger.info("CLE Server exiting.")
