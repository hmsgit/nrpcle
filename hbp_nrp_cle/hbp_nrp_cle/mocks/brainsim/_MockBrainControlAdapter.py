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
'''
MockControlAdapter.py
moduleauthor: Michael.Weber@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import IBrainControlAdapter

__author__ = 'Michael Weber'


class MockBrainControlAdapter(IBrainControlAdapter):
    """
    Represents a mocked controller object for the neuronal simulator
    """
    #This is a mock, there might be unused parameters
    #pylint: disable=W0613
    def __init__(self):
        """
        Initializes the Mock control adapter
        """
        self.__is_initialized = False
        self.__is_alive = False
        self.__rank = None
        self.detector_devices = []
        self.generator_devices = []

    def get_populations(self):
        """
        Gets an information about the populations currently available

        :return: A list of population infos
        """
        return []

    def initialize(self, **params):
        """
        Initializes the neuronal simulator
        :param params: A dictionary of configuration parameters
        """
        self.__is_initialized = True
        self.__rank = 0

        print "Setting up MOCK."
        return self.__is_initialized

    def is_alive(self):  # -> bool:
        """
        Gets a status whether the neuronal simulator is still alive
        :return: True if the simulator is alive, otherwise False
        """
        return self.__is_alive

    def run_step(self, dt):  # -> None:
        """
        Runs the neuronal simulator for the given amount of simulated time
        :param dt: the simulated time in milliseconds
        """

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        self.__is_alive = False

    def reset(self):  # -> None:
        """
        Resets the neuronal simulator
        """
        pass

    def load_brain(self, network_file, **populations):
        """
        Loads the neuronal network contained in the given file

        :param network_file: The path to the neuronal network file
        :param populations: The populations to create
        """
        self.__is_alive = True

    @staticmethod
    def get_Timeout():  # -> None:
        """
        returns The maximum amount of time (in seconds) to wait for the end of this step
        """
        return None
