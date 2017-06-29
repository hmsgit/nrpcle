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
This module represents the interface for a transfer functions manager
"""

__author__ = 'GeorgHinkel'


class ITransferFunctionManager(object):  # pragma: no cover
    """
    Represents the interface of a transfer functions node
    """

    def run_neuron_to_robot(self, t):  # -> None:
        """
        Runs the transfer functions from the neuronal simulator towards the robot

        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation

        :param t:  The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self, name):  # -> None:
        """
        Initializes the transfer Function node with the given name

        :param name: The name for this transfer function node
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize_tf(self, tf):
        """
        Initializes the given transfer function

        This method is used if a transfer function is replaced after initialization of the tf
        manager

        :param name: The transfer function
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self):  # -> None:
        """
        Resets the transfer functions
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def transfer_functions(self):
        """
        Gets a list of transfer functions managed by this instance

        :return: A list of transfer functions
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def shutdown(self):
        """
        Shuts down the Transfer Function manager
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def hard_reset_robot_devices(self):
        """
        Performs a hard reset for the devices that connect with the simulated robot
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def hard_reset_brain_devices(self):
        """
        Performs a hard reset for the devices that connect with the neuronal simulation
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def brain_adapter(self):
        """
        Gets or sets the adapter to the brain simulation
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
