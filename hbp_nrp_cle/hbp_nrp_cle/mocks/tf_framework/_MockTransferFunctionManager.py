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
This module contains the mock implementation of the transfer functions manager
"""

from hbp_nrp_cle.tf_framework._TransferFunctionInterface import ITransferFunctionManager
import time

__author__ = 'GeorgHinkel'


class MockTransferFunctionManager(ITransferFunctionManager):
    """
    Mocks the transfer functions manager
    """

    def __init__(self):
        """
        Initializes a new transfer functions mock
        """
        self.__name = None
        self.__r2nTimes = []
        self.__n2rTimes = []
        self.__sleepTime = 1
        self.__publish_error_callback = None

    def initialize(self, name):
        """
        Initializes the mocked transfer functions

        :param name: The user friendly name of this manager
        """
        self.__name = name

    def initialize_tf(self, tf):
        """
        Initializes the given transfer function

        This method is used if a transfer function is replaced after initialization of the tf
        manager

        :param name: The transfer function
        """
        pass

    def run_neuron_to_robot(self, t):
        """
        Runs the transfer function mocks for neuron to robot direction

        :param t: The simulation time of the neuronal simulator
        """
        self.__n2rTimes.append(t)
        time.sleep(self.__sleepTime)

    def run_robot_to_neuron(self, t):
        """
        Runs the transfer function mocks for the robot to neuron direction

        :param t: The simulation time of the world simulator
        """
        self.__r2nTimes.append(t)
        time.sleep(self.__sleepTime)

    @property
    def name(self):
        """
        Gets the name of the current transfer function manager mock
        """
        return self.__name

    @property
    def is_initialized(self):
        """
        Gets a value indicating whether the transfer function manager has been initialized
        """
        return self.__name is not None

    @property
    def robot_to_neuron_times(self):
        """
        Gets the times the transfer function manager has been called from robot to neurons
        """
        return self.__r2nTimes

    @property
    def neuron_to_robot_times(self):
        """
        Gets the times the transfer function manager has been called from neuron to robot
        """
        return self.__n2rTimes

    def transfer_functions(self):
        """
        Gets the list of Transfer Functions managed by this mock

        :return: An empty list
        """
        return []

    @property
    def sleep_time(self):
        """
        Gets the sleep_time of the mock when the transfer functions are called
        """
        return self.__sleepTime

    @sleep_time.setter
    def sleep_time(self, sleep_time):
        """
        Sets the sleep time when the transfer functions are called

        :param sleep_time: The new sleep time for this mock
        """
        self.__sleepTime = sleep_time

    def reset(self):  # -> None:
        """
        Resets the transfer functions
        """
        pass

    def shutdown(self):
        """
        Shutdown the tf manager
        """
        pass

    def hard_reset_robot_devices(self):
        """
        Performs a hard reset for the devices that connect with the simulated robot
        """
        pass

    def hard_reset_brain_devices(self):
        """
        Performs a hard reset for the devices that connect with the neuronal simulation
        """
        pass
