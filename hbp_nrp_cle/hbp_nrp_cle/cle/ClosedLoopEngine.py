"""
Implementation of the closed loop engine.
"""

__author__ = 'LorenzoVannucci'

import threading
import time
from hbp_nrp_cle.cle.CLEInterface import IClosedLoopControl
from hbp_nrp_cle.brainsim import IBrainCommunicationAdapter, IBrainControlAdapter
from hbp_nrp_cle.robotsim import IRobotCommunicationAdapter, IRobotControlAdapter
from hbp_nrp_cle.tf_framework import ITransferFunctionManager
from hbp_nrp_cle.cle.__helper import get_tf_elapsed_times
import logging

logger = logging.getLogger(__name__)


class ControlThread(threading.Thread):
    """
    Separate thread to control a simulation.
    """

    def __init__(self, control_adapter, end_flag):
        """
        Constructor

        :param control_adapter: the adapter (able to do run_step)
        :param endFlag: the flag on wich the CLE waits
                        for the simulation to end
        """
        super(ControlThread, self).__init__()
        self.daemon = True

        self.ctrlad = control_adapter
        self.timestep = 0.0
        self.end_flag = end_flag

        self.start_flag = threading.Event()
        self.start_flag.clear()

    def run_step(self, timestep):
        """
        Makes the simulation run.

        :param timestep: the simulation time in seconds
        """
        self.timestep = timestep
        self.start_flag.set()

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        while True:
            self.start_flag.wait()
            # print 'Simulating...'
            self.ctrlad.run_step(self.timestep)
            self.start_flag.clear()
            self.end_flag.set()


class TransferFunctionsThread(threading.Thread):
    """
    Thread running the tranfer functions.
    """

    def __init__(self, transfer_function_manager, robot_comm_adapter, brain_comm_adapter, end_flag):
        """
        Constructor

        :param transfer_function_manager: the adapter (able to do run_step)
        :param robot_comm_adapter: The robot comm adapter
        :param brain_comm_adapter: The brain comm adapter
        :param endFlag: the flag on wich the CLE waits
                        for the computation to end
        """
        super(TransferFunctionsThread, self).__init__()
        self.daemon = True

        assert isinstance(transfer_function_manager, ITransferFunctionManager)
        assert isinstance(robot_comm_adapter, IRobotCommunicationAdapter)
        assert isinstance(brain_comm_adapter, IBrainCommunicationAdapter)

        self.tfm = transfer_function_manager
        self.rcm = robot_comm_adapter
        self.bcm = brain_comm_adapter
        self.simtime = 0.0
        self.end_flag = end_flag

        self.start_flag = threading.Event()
        self.start_flag.clear()

    def run_step(self, simtime):
        """
        Makes the simulation run.

        :param simtime: the simulation time in seconds
        """
        self.simtime = simtime
        self.start_flag.set()

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        while True:
            self.start_flag.wait()
            t = self.simtime
            self.bcm.refresh_buffers(t)
            self.rcm.refresh_buffers(t)
            self.tfm.run_neuron_to_robot(t)
            self.tfm.run_robot_to_neuron(t)
            self.start_flag.clear()
            self.end_flag.set()


# pylint: disable=R0902
# the attributes are reasonable in this case
class ClosedLoopEngine(IClosedLoopControl, threading.Thread):
    """
    Implementation of the closed loop engine.
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
        :param brain_comm_adapter: an instance of IBraimCommunicationAdapter
        :param transfer_function_manager: an instance of ITransferFunctionManager
        :param dt: The CLE time step in seconds
        """
        super(ClosedLoopEngine, self).__init__()
        self.daemon = True

        # set up the robot control adapter thread
        assert isinstance(robot_control_adapter, IRobotControlAdapter)
        self.rca = robot_control_adapter
        assert isinstance(robot_comm_adapter, IRobotCommunicationAdapter)
        self.rcm = robot_comm_adapter
        self.rct_flag = threading.Event()
        self.rct = ControlThread(self.rca, self.rct_flag)
        self.rct.start()
        logger.info("robot control adapter ready")

        # set up the brain control adapter thread
        assert isinstance(brain_control_adapter, IBrainControlAdapter)
        self.bca = brain_control_adapter
        assert isinstance(brain_comm_adapter, IBrainCommunicationAdapter)
        self.bcm = brain_comm_adapter
        self.bct_flag = threading.Event()
        self.bct = ControlThread(self.bca, self.bct_flag)
        self.bct.start()
        logger.info("brain control adapter ready")

        # set up the transfer function thread
        assert isinstance(transfer_function_manager, ITransferFunctionManager)
        self.tfm = transfer_function_manager
        self.tft_flag = threading.Event()
        self.tft = TransferFunctionsThread(self.tfm, robot_comm_adapter, brain_comm_adapter,
                                           self.tft_flag)
        self.tft.start()
        logger.info("transfer function ready")

        # default timestep
        self.timestep = dt

        # main thread control
        self.started = False
        self.stop_flag = threading.Event()
        self.stop_flag.set()
        logger.info("CLE started")

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

    def initialize(self):
        """
        Initializes the closed loop engine.
        """
        self.rca.initialize()
        self.bca.initialize()
        self.tfm.initialize('tfnode')
        self.clock = 0.0
        self.initialized = True
        self.running = False
        self.start_time = 0.0
        self.elapsed_time = 0.0
        logger.info("CLE initialized")

    @property
    def is_initialized(self):
        """
        Returns True if the simulation is initialized, False otherwise.
        """
        return self.initialized

    def run_step(self, timestep):
        """
        Runs both simulations for the given time step in seconds.

        :param timestep: simulation time, in seconds
        :return: Updated simulation time, otherwise -1
        """
        self.running_flag.clear()

        # robot simulation
        self.rct_flag.clear()
        if not self.rca.is_alive:
            return -1

        start = time.time()
        self.rct.run_step(timestep)
        self.__rca_elapsed_time += time.time() - start

        # brain simulation
        self.bct_flag.clear()
        if not self.bca.is_alive():
            return -1

        start = time.time()
        self.bct.run_step(timestep * 1000.0)
        self.__bca_elapsed_time += time.time() - start

        # transfer functions
        self.tft_flag.clear()
        self.tft.run_step(self.clock)

        # update clock
        self.clock += timestep

        # wait for all thread to finish
        self.rct_flag.wait()
        self.bct_flag.wait()
        self.tft_flag.wait()

        self.running_flag.set()
        return self.clock

    def shutdown(self):
        """
        Shuts down both simulations.
        """
        self.rcm.shutdown()
        self.bcm.shutdown()
        self.rca.shutdown()
        self.bca.shutdown()
        logger.info("simulations shutdown")

    def start(self):
        """
        Starts the orchestrated simulations.
        """
        self.start_time = time.time()
        self.running = True
        if not self.started:
            self.started = True
            threading.Thread.start(self)
        else:
            self.stop_flag.set()
        logger.info("simulation started")

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        while True:
            self.run_step(self.timestep)
            self.stop_flag.wait()

    def stop(self):
        """
        Stops the orchestrated simulations. Also waits for the current
        simulation step to end.
        """
        self.stop_flag.clear()
        self.wait_step()
        self.running = False
        self.elapsed_time += time.time() - self.start_time
        logger.info("simulation stopped")

    def reset(self):
        """
        Reset the orchestrated simulations.
        """
        self.stop()
        self.wait_step()
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

    def wait_step(self):
        """
        Wait for the currently running simulation step to end.
        """
        self.running_flag.wait()
