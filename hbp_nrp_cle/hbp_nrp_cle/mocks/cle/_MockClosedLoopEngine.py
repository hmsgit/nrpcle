# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
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

        self.running = False
        self.start_time = 0.0
        self.elapsed_time = 0.0

        self.initialized = False

    def initialize(self, network_file, **configuration):
        """
        Initializes the closed loop engine.

        :param configuration: A set of populations
        containing the neural network definition
        :param network_file: A python PyNN script or an h5 file
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

        :param timestep: The CLE time step in seconds
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
        self.start_time = time.time()
        self.running = True
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

    def stop(self, forced=False):
        """
        Stops the orchestrated simulations. Also waits for the current
        simulation step to end.

        :param forced: If set, the CLE instance cancels pending tasks
        """
        self.stop_flag.clear()
        self.wait_step()
        self.running = False
        self.elapsed_time += time.time() - self.start_time

    def reset(self):
        """
        Reset the orchestrated simulations.
        """
        self.stop()
        self.wait_step()
        self.clock = 0.0
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self.running = False

    @property
    def simulation_time(self):  # -> float64
        """
        Get the current simulation time.
        """
        return self.clock

    def reset_world(self, sdf_world_string=None):
        """
        Reset the world to a given configuration, or to the initial one if none is passed.

        :param sdf_world_string: the new environment to be set in the world simulator
            (default value is the empty string for compatibility with the ROS message).
        """
        self.stop()
        self.running = False

    @property
    def real_time(self):  # -> float64
        """
        Get the current simulation time.
        """
        if self.running:
            return self.elapsed_time + time.time() - self.start_time
        return self.elapsed_time

    def tf_elapsed_time(self):
        """
        Gets the time share of the Transfer Functions
        """
        return {}

    def brainsim_elapsed_time(self):
        """
        Gets the time share of the brain simulation
        """
        return 0.0

    def robotsim_elapsed_time(self):
        """
        Gets the time share of the robot simulation
        """
        return 0.0

    def wait_step(self, timeout=None):
        """
        Wait for the currently running simulation step to end.

        :param timeout: The maximum amount of time (in seconds) to wait for the end of this step
        """
        self.running_flag.wait(timeout=timeout)

    def load_network_from_file(self, network_file, **network_configuration):

        """
        Load (or reload) the brain model from a file the neuronal network file

        :param network_file: A python PyNN script or an h5 file
        containing the neural network definition
        :param network_configuration: A dictionary indexed by population names and
        containing neuron indices. Neuron indices could be defined a single integer,
        list of integers or python slices. Python slices could be replaced by a
        dictionary containing the 'from', 'to' and 'step' values.
        """
        return
