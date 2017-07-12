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
This module contains mock implementations for a spike generator device which record the changes
to the device parameters and can be used to track transfer function activity without the need to
run an entire brain simulation.
'''


from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, IPoissonSpikeGenerator

__author__ = 'PatrikScheidecker'


class MockSpikeGenerator(AbstractMockBrainDevice, IFixedSpikeGenerator):
    """
    Mock device base class for spike generators.
    """

    default_parameters = {
        "target": "excitatory",
        "rate": 1.0
    }

    # pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param rate: Rate of spike generation
        """
        super(MockSpikeGenerator, self).__init__(**params)

        self.__rate = self._parameters["rate"]
        self.__history = []

    @property
    def rate(self):
        """
        Returns the spike generation rate
        """
        return self.__rate

    @rate.setter
    def rate(self, rate):
        """
        Sets the spike generation rate

        :param rate: float
        """
        self.__rate = rate
        self.__history.append(rate)

    @property
    def history(self):
        """
        Lists the rates assigned to this device

        :return: A list of float values
        """
        return self.__history


class MockFixedSpikeGenerator(MockSpikeGenerator, IFixedSpikeGenerator):
    """
    Mock device representing a Fixed Spike Generator, which generates spikes with a
    fixed predefined interval duration.
    """


class MockPoissonSpikeGenerator(MockSpikeGenerator, IPoissonSpikeGenerator):
    """
    Mock device representing a Poisson Spike Generator, which generates spikes with according
    to a poisson distribution.
    """
