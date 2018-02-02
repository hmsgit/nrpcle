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
This module contains common definitions for mapping specifications
"""

__author__ = 'georghinkel'

from ._TransferFunction import TransferFunction


class ParameterMappingSpecification(object):
    """
    Defines a parameter mapping for a transfer function
    """

    def __init__(self, name):
        self.__name = name

    def __call__(self, transfer_function):  # -> object:
        """
        Applies the parameter mapping to the given transfer function
        """
        if isinstance(transfer_function, TransferFunction):
            parameters = transfer_function.params
            for i in range(0, len(parameters)):
                if parameters[i] == self.name:
                    parameters[i] = self
                    return transfer_function
        else:
            if callable(transfer_function):
                return self.__call__(self.create_tf()(transfer_function))
            raise Exception("Can only map parameters for functions or transfer functions")
        raise Exception(
            "Could not map parameter as no parameter with the name " + self.name + " exists")

    def create_adapter(self, tf_manager):
        """
        Creates the adapter for this mapping operator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def create_tf(self):
        """
        Creates a TF in case the TF specification has been omitted
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__name

    # pylint: disable=no-self-use
    @property
    def is_brain_connection(self):
        """
        Returns whether the the parameter is connected to the neuronal network

        :return: True, if the parameter is mapped to the neuronal network, otherwise False
        """
        return False

    # pylint: disable=no-self-use
    @property
    def is_robot_connection(self):
        """
        Returns whether the parameter is connected to the simulated robot

        :return: True, if the parameter is mapped to the simulated robot, otherwise False
        """
        return False
