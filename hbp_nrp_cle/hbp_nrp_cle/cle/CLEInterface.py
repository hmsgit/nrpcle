# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
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
This module represents the interfaces for the closed loop engine.
"""

__author__ = 'LorenzoVannucci'


class ForcedStopException(Exception):
    """
    Represents that a closed loop has been forced to quit
    """
    def __init__(self):
        super(ForcedStopException, self).__init__("The simulation was forced to stop")


# pylint: disable=R0921
class IClosedLoopControl(object):  # pragma: no cover
    """
    Represents the closed loop engine synchronization mechanism.
    """
    def load_network_from_file(self, network_file, **network_configuration):
        """
        Load (or reload) the brain model from a file the neuronal network file

        :param network_file: A python PyNN script or an h5 file
          containing the neural network definition
        :param network_configuration: A dictionary indexed by population names and
          containing neuron indices. Neuron indices can be defined by a single integer,
          list of integers or python slices. Python slices can be replaced by a
          dictionary containing the 'from', 'to' and 'step' values.
        """
        raise NotImplementedError("Method not implemented")

    def initialize(self, network_file, configuration):  # -> None:
        """
        Initializes the closed loop engine.

        :param network_file: A python PyNN script or an h5 file
         containing the neural network definition
        :param configuration: A set of populations
        """
        raise NotImplementedError("Method not implemented")

    @property
    def is_initialized(self):  # -> bool
        """
        Returns True if the simulation is initialized, False otherwise.
        """
        raise NotImplementedError("Method not implemented")

    def run_step(self, timestep):  # -> float64:
        """
        Runs both simulations for the given time step in seconds.

        :param timestep: The CLE time step in seconds
        :return: Updated simulation time, otherwise -1
        """
        raise NotImplementedError("Method not implemented")

    def shutdown(self):  # -> None:
        """
        Shuts down both simulations.
        """
        raise NotImplementedError("Method not implemented")

    def start(self):  # -> None:
        """
        Starts the orchestrated simulations.
        """
        raise NotImplementedError("Method not implemented")

    def stop(self, forced=False):  # -> None:
        """
        Stops the orchestrated simulations. Also waits for the current
        simulation step to end.

        :param forced: If set, the CLE instance cancels pending tasks
        """
        raise NotImplementedError("Method not implemented")

    def reset(self):  # -> None:
        """
        Reset the orchestrated simulations.
        """
        raise NotImplementedError("Method not implemented")

    def reset_world(self, sdf_world_string):
        """
        Reset the world to the configuration described by sdf_world_string.
        """
        raise NotImplementedError("Method not implemented")

    @property
    def simulation_time(self):  # -> float64
        """
        Get the current simulation time.
        """
        raise NotImplementedError("Method not implemented")

    @property
    def real_time(self):  # -> float64
        """
        Get the total execution time.
        """
        raise NotImplementedError("Method not implemented")

    def tf_elapsed_time(self):
        """
        Gets the time share of the Transfer Functions
        """
        raise NotImplementedError("Method not implemented")

    def brainsim_elapsed_time(self):
        """
        Gets the time share of the brain simulation
        """
        raise NotImplementedError("Method not implemented")

    def robotsim_elapsed_time(self):
        """
        Gets the time share of the robot simulation
        """
        raise NotImplementedError("Method not implemented")

    def wait_step(self, timeout=None):  # -> None
        """
        Wait for the currently running simulation step to end.

        :param timeout: The maximum amount of time (in seconds) to wait for the end of this step
        """
        raise NotImplementedError("Method not implemented")
