"""
Implementation of the closed loop engine.
"""

__author__ = 'LorenzoVannucci'

import time
import threading
from hbp_nrp_cle.cle.CLEInterface import IClosedLoopControl


class MockClosedLoopEngine(IClosedLoopControl,
                           threading.Thread):  # pragma: no cover
    """
    Implementation of the closed loop engine.
    """

    def __init__(self, dt):
        """
        Create an instance of the cle.
        :param dt: The CLE time step in seconds
        """
        super(MockClosedLoopEngine, self).__init__()
        self.daemon = True

        self.timestep = dt

        # synch thread
        self.started = False
        self.stop_flag = threading.Event()
        self.stop_flag.set()
        self.running_flag = threading.Event()
        self.running_flag.set()

        self.clock = 0.0

        self.initialized = False

    def initialize(self):
        """
        Initializes the closed loop engine.
        """
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
        :param dt: The CLE time step in seconds
        :return: Updated simulation time, otherwise -1
        """
        self.running_flag.clear()
        time.sleep(timestep)
        self.clock += timestep
        self.running_flag.set()
        return self.clock

    def shutdown(self):
        """
        Shuts down both simulations.
        """
        pass

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
