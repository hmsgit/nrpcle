"""
Implementation of the closed loop engine.
"""

__author__ = 'LorenzoVannucci'

import threading
from hbp_nrp_cle.cle.CLEInterface import IClosedLoopControl


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


class TransferFunctionsThread(threading.Thread):
    """
    Thread running the tranfer functions.
    """

    def __init__(self, transferfunctionmanager, end_flag):
        """
        Constructor
        :param controladapter: the adapter (able to do run_step)
        :param endFlag: the flag on wich the CLE waits
                        for the computation to end
        """
        super(TransferFunctionsThread, self).__init__()
        self.daemon = True

        self.tfm = transferfunctionmanager
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
            self.tfm.run_neuron_to_robot(self.simtime)
            self.tfm.run_robot_to_neuron(self.simtime)
            self.start_flag.clear()
            self.end_flag.set()


# pylint: disable=R0902
# the attributes are reasonable in this case
class ClosedLoopEngine(IClosedLoopControl, threading.Thread):
    """
    Implementation of the closed loop engine.
    """

    def __init__(self,
                 robotcontroladapter,
                 braincontroladapter,
                 transferfunctionmanager,
                 dt):
        """
        Create an instance of the cle.
        :param robotcontroladapter: an instance of IRobotContolAdapter
        :param braincontroladapter: an instance of IBrainContolAdapter
        :param transferfunctionmanager: an instance of ITransferFunctionManager
        :param dt: The CLE time step in seconds
        """
        super(ClosedLoopEngine, self).__init__()
        self.daemon = True

        # set up the robot control adapter thread
        self.rca = robotcontroladapter
        self.rct_flag = threading.Event()
        self.rct = ControlThread(self.rca, self.rct_flag)
        self.rct.start()

        # set up the brain control adapter thread
        self.bca = braincontroladapter
        self.bct_flag = threading.Event()
        self.bct = ControlThread(self.bca, self.bct_flag)
        self.bct.start()

        # set up the transfer function thread
        self.tfm = transferfunctionmanager
        self.tft_flag = threading.Event()
        self.tft = TransferFunctionsThread(self.tfm, self.tft_flag)
        self.tft.start()

        # default timestep
        self.timestep = dt

        # main thread control
        self.started = False
        self.stop_flag = threading.Event()
        self.stop_flag.set()

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
        self.bct_flag.clear()
        if not self.bca.is_alive():
            return -1
        self.bct.run_step(timestep * 1000.0)

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
        self.rca.shutdown()
        self.bca.shutdown()

    def start(self):
        """
        Starts the orchestrated simulations.
        """
        if not self.started:
            self.started = True
            threading.Thread.start(self)
        else:
            self.stop_flag.set()

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        while True:
            self.run_step(self.timestep)
            self.stop_flag.wait()

    def stop(self):
        """
        Stops the orchestrated simulations.
        """
        self.stop_flag.clear()

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
