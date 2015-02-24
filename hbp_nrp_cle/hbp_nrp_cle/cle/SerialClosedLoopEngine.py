"""
Implementation of the closed loop engine.
"""

__author__ = 'LorenzoVannucci'

import threading
from hbp_nrp_cle.cle.CLEInterface import IClosedLoopControl
from hbp_nrp_cle.tf_framework import ITransferFunctionManager
from hbp_nrp_cle.brainsim import IBrainCommunicationAdapter, IBrainControlAdapter
from hbp_nrp_cle.robotsim import IRobotCommunicationAdapter, IRobotControlAdapter


class ControlThread(threading.Thread):
    """
    Separate thread to control a simulation.
    """

    def __init__(self, controladapter, end_flag):
        """
        Constructor
        :param controladapter: the adapter (able to do run_step)
        :param endFlag: the flag on wich the CLE waits
                        for the simulation to end
        """
        super(ControlThread, self).__init__()
        self.daemon = True

        self.ctrlad = controladapter
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
#            print 'Simulating...'
            self.ctrlad.run_step(self.timestep)
            self.start_flag.clear()
            self.end_flag.set()


# pylint: disable=R0902
# the attributes are reasonable in this case
class SerialClosedLoopEngine(IClosedLoopControl):
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
        Create an instance of the serial cle.
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
        # set up the robot control adapter thread
        self.rca = robot_control_adapter
        self.rct_flag = threading.Event()
        self.rct = ControlThread(self.rca, self.rct_flag)
        self.rct.start()
        self.rcm = robot_comm_adapter

        # set up the brain control adapter thread
        self.bca = brain_control_adapter
        self.bcm = brain_comm_adapter

        # set up the transfer function thread
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

    def initialize(self):
        """
        Initializes the closed loop engine.
        """
        self.rca.initialize()
        self.bca.initialize()
        self.tfm.initialize('tfnode')
        self.clock = 0.0
        self.initialized = True

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
        self.rct.run_step(timestep)

        # brain simulation
        self.bca.run_step(timestep * 1000.0)

        # transfer functions
        clk = self.clock
        self.bcm.refresh_buffers(clk)
        self.rcm.refresh_buffers(clk)
        self.tfm.run_neuron_to_robot(clk)
        self.tfm.run_robot_to_neuron(clk)

        # update clock
        self.clock += timestep

        # wait for all thread to finish
        self.rct_flag.wait()

        self.running_flag.set()
        return self.clock

    def shutdown(self):
        """
        Shuts down both simulations.
        """
        self.stop_flag.set()
        self.stopped_flag.wait(5)
        self.tfm.shutdown()
        self.rca.shutdown()
        self.bca.shutdown()

    def start(self):
        """
        Starts the orchestrated simulations.
        This function does not return (starts an infinite loop).
        """
        self.stop_flag.clear()
        self.rca.unpause()
        self.stopped_flag.clear()
        while not self.stop_flag.isSet():
            self.run_step(self.timestep)
        self.rca.pause()
        self.stopped_flag.set()

    def stop(self):
        """
        Stops the orchestrated simulations.
        Must be called on a separate thread from the start one, as an example
        by a threading.Timer.
        """
        self.stop_flag.set()

    def reset(self):
        """
        Reset the orchestrated simulations (stops them before resetting).
        """
        self.stop()
        self.wait_step()
        self.rca.reset()
        self.bca.reset()
        self.tfm.reset()
        self.clock = 0.0

    @property
    def time(self):
        """
        Get the current simulation time.
        """
        return self.clock

    def wait_step(self):
        """
        Wait for the currently running simulation step to end.
        """
        self.running_flag.wait()
