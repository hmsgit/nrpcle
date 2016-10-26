"""
Implementation of the closed loop engine.
"""

__author__ = 'LorenzoVannucci'

import time
import logging
import threading
from hbp_nrp_cle.cle.CLEInterface import IClosedLoopControl, ForcedStopException
from hbp_nrp_cle.tf_framework import ITransferFunctionManager
from hbp_nrp_cle.brainsim import IBrainCommunicationAdapter, IBrainControlAdapter
from hbp_nrp_cle.robotsim import IRobotCommunicationAdapter, IRobotControlAdapter
from hbp_nrp_cle.cle.__helper import get_tf_elapsed_times
from hbp_nrp_cle.robotsim.GazeboHelper import GazeboHelper

logger = logging.getLogger('hbp_nrp_cle')


# pylint: disable=R0902
# the attributes are reasonable in this case
class ClosedLoopEngine(IClosedLoopControl):
    """
    Implementation of the closed loop engine that overcomes the NEST
    segmentation fault problem by serializing the transfer function execution
    and the brain simulation.
    """

    def __init__(self,
                 robot_control_adapter,
                 robot_comm_adapter,
                 brain_control_adapter,
                 brain_comm_adapter,
                 transfer_function_manager,
                 dt):
        """
        Create an instance of the cle.

        :param robot_control_adapter: an instance of IRobotContolAdapter
        :param robot_comm_adapter: an instance of IRobotCommunicationAdapter
        :param brain_control_adapter: an instance of IBrainContolAdapter
        :param brain_comm_adapter: an instance of IBrainCommunicationAdapter
        :param transfer_function_manager: an instance of ITransferFunctionManager
        :param dt: The CLE time step in seconds
        """

        assert isinstance(robot_control_adapter, IRobotControlAdapter)
        assert isinstance(robot_comm_adapter, IRobotCommunicationAdapter)
        assert isinstance(brain_control_adapter, IBrainControlAdapter)
        assert isinstance(brain_comm_adapter, IBrainCommunicationAdapter)
        assert isinstance(transfer_function_manager, ITransferFunctionManager)

        self.rca = robot_control_adapter
        self.rca_future = None
        self.rcm = robot_comm_adapter

        self.bca = brain_control_adapter
        self.bcm = brain_comm_adapter

        self.tfm = transfer_function_manager

        # default timestep
        self.timestep = dt

        # stop flag ask the main loop to stop
        self.stop_flag = threading.Event()
        self.stop_flag.clear()

        # stopped flag indicate if the main loop is stopped
        self.stopped_flag = threading.Event()
        self.stopped_flag.set()

        # step wait
        self.running_flag = threading.Event()
        self.running_flag.set()

        # global clock
        self.clock = 0.0

        self.initialized = False

        # time measurements
        # The real time is measured this way: each time play is called
        # start_time is set to the current clock time. Each time the stop
        # function is called (pausing the simulation) elapsed_time is
        # incremented with (stop time - start_time). Thus, the real time is
        # elapsed_time                               if paused
        # elapsed_time + (current time - start_time) if running
        self.running = False
        self.start_time = 0.0
        self.elapsed_time = 0.0

        self.__rca_elapsed_time = 0.0
        self.__bca_elapsed_time = 0.0
        self.__network_file = None
        self.__network_configuration = None

        self.__initial_robot_pose = None

        # This is to be eventually removed, as communications towards gazebo should
        # be done from inside the RosControlAdapter
        self.gazebo_helper = GazeboHelper()

        self.initial_models = None
        self.initial_lights = None

    def initialize(self, network_file, **configuration):
        """
        Initializes the closed loop engine.
        :param network_file: A python PyNN script or an h5 file
         containing the neural network definition
        :param configuration: A set of populations
        """
        self.rca.initialize()
        self.bca.initialize()
        self.__network_file = network_file
        self.__network_configuration = configuration
        self.bca.load_brain(network_file, **configuration)
        self.tfm.initialize('tfnode')
        self.clock = 0.0
        self.running = False
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self.initialized = True

    @property
    def is_initialized(self):
        """
        Returns True if the simulation is initialized, False otherwise.
        """
        return self.initialized

    def load_network_from_file(self, network_file, **network_configuration):
        """
        Creates a new brain in the running simulation

        :param network_file: A python PyNN script or an h5 file
        containing the neural network definition
        :param network_configuration: A dictionary indexed by population names and
        containing neuron indices. Neuron indices can be defined by
        lists of integers or slices. Slices are either python slices or
        dictionaries containing 'from', 'to' and 'step' values.
        """
        if self.initialized:
            if self.running:
                self.stop()
            if self.bca.is_alive():
                self.bca.shutdown()
            logger.info("Recreating brain from file " + network_file)
            self.bca.load_brain(network_file, **network_configuration)
            logger.info("Resetting TFs")
            self.tfm.hard_reset_brain_devices()

    def run_step(self, timestep):
        """
        Runs both simulations for the given time step in seconds.

        :param timestep: simulation time, in seconds
        :return: Updated simulation time, otherwise -1
        """
        self.running_flag.clear()
        clk = self.clock

        # robot simulation
        logger.debug("Run step: Robot simulation.")
        start = time.time()
        self.rca_future = self.rca.run_step_async(timestep)
        self.rcm.refresh_buffers(clk)
        self.__rca_elapsed_time += time.time() - start

        # brain simulation
        logger.debug("Run step: Brain simulation")
        start = time.time()
        self.bca.run_step(timestep * 1000.0)
        self.bcm.refresh_buffers(clk)
        self.__bca_elapsed_time += time.time() - start

        # transfer functions
        logger.debug("Run step: Transfer functions")
        self.tfm.run_robot_to_neuron(clk)
        self.tfm.run_neuron_to_robot(clk)

        # update clock
        self.clock += timestep

        # wait for all thread to finish
        logger.debug("Run_step: waiting on Control thread")
        try:
            self.rca_future.result()
        except ForcedStopException:
            logger.warn("Simulation was brutally stopped.")

        self.running_flag.set()
        logger.debug("Run_step: done !")
        return self.clock

    def shutdown(self):
        """
        Shuts down both simulations.
        """
        self.stop_flag.set()
        self.stopped_flag.wait(5)
        self.rcm.shutdown()
        self.bcm.shutdown()
        self.rca.shutdown()
        self.bca.shutdown()

    def start(self):
        """
        Starts the orchestrated simulations.
        This function does not return (starts an infinite loop).
        """
        self.stop_flag.clear()
        self.stopped_flag.clear()
        self.start_time = time.time()
        self.running = True
        while not self.stop_flag.isSet():
            self.run_step(self.timestep)
        self.running = False
        self.elapsed_time += time.time() - self.start_time
        self.stopped_flag.set()

    def stop(self, forced=False):
        """
        Stops the orchestrated simulations. Also waits for the current
        simulation step to end.
        Must be called on a separate thread from the start one, as an example
        by a threading.Timer.

        :param forced: If set, the CLE instance cancels pending tasks
        :except Exception: throws an exception if the CLE was forced to stop but could not be
        stopped
        """
        self.stop_flag.set()
        if forced:
            if self.rca_future is not None and self.rca_future.running():
                self.running_flag.wait(5)
                if self.rca_future.running():
                    self.rca_future.set_exception(ForcedStopException())
            self.wait_step(timeout=5)
            if not self.running_flag.isSet():
                raise Exception("The simulation loop could not be completed")
        else:
            self.wait_step()

    def reset(self):
        """
        Reset the orchestrated simulations (stops them before resetting).
        """
        self.stop()
        self.rca.reset()
        self.bca.reset()
        self.tfm.reset()
        self.clock = 0.0
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self.running = False
        self.__rca_elapsed_time = 0.0
        self.__bca_elapsed_time = 0.0
        logger.info("CLE reset")

    def reset_world(self, sdf_world_string=""):
        """
        Reset the world to a given configuration, or to the initial one if none is passed.

        :param sdf_world_string: the new environment to be set in the world simulator
            (default value is the empty string for compatibility with the ROS message).
        """
        if len(sdf_world_string) > 0:
            models, lights = self.gazebo_helper.parse_world_file(sdf_world_string)
        else:
            # backward compatibility
            # when working without collab, reset to the initial state
            models = self.initial_models
            lights = self.initial_lights

        self.stop()
        self.rca.reset_world(models, lights)
        self.running = False
        logger.info("CLE world reset")

    def reset_brain(self, network_file=None, network_configuration=None):
        """
        Reloads the brain and resets the transfer function.
        If no parameter is specified, it reloads the initial brain.

        :param network_file: A python PyNN script containing the neural network definition
        :param network_configuration: A set of populations
        """
        if network_file is not None and network_configuration is not None:
            self.load_network_from_file(network_file, **network_configuration)
        else:
            self.load_network_from_file(self.__network_file, **self.__network_configuration)
        logger.info("CLE Brain reset")

    @property
    def simulation_time(self):  # -> float64
        """
        Get the current simulation time.
        """
        return self.clock

    @property
    def real_time(self):  # -> float64
        """
        Get the total execution time.
        """
        if self.running:
            return self.elapsed_time + time.time() - self.start_time
        return self.elapsed_time

    @property
    def initial_robot_pose(self):
        """
        Get the robot pose at the beginning of the simulation.
        """
        return self.__initial_robot_pose

    @initial_robot_pose.setter
    def initial_robot_pose(self, value):
        """
        Set the robot pose at the beginning of the simulation.
        Usually performed after CLE initialization.
        :param value: The new value for the initial robot pose.
        """
        self.__initial_robot_pose = value

    def reset_robot_pose(self):
        """
        Set the robot in the simulation to its initial pose.
        """
        self.gazebo_helper.set_model_pose('robot', self.__initial_robot_pose)

    def tf_elapsed_time(self):
        """
        Gets the time share of the Transfer Functions
        """
        return get_tf_elapsed_times(self.tfm)

    def brainsim_elapsed_time(self):
        """
        Gets the time share of the brain simulation
        """
        return self.__bca_elapsed_time

    def robotsim_elapsed_time(self):
        """
        Gets the time share of the robot simulation
        """
        return self.__rca_elapsed_time

    def wait_step(self, timeout=None):
        """
        Wait for the currently running simulation step to end.

        :param timeout: The maximum amount of time (in seconds) to wait for the end of this step
        """
        self.running_flag.wait(timeout)
