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
This module contains the mock implementations for a noisy current source which records the
changes to the supported device parameters and can be used to track transfer function activity.
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import INCSource

__author__ = 'MichaelWeber'


class MockNCSource(AbstractMockBrainDevice, INCSource):
    """
    Represents a Mock of a noisy current generator
    """

    default_parameters = {
        "mean": 1.0
    }

    #pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        """
        super(MockNCSource, self).__init__(**params)

        self.__mean = self._parameters["mean"]
        self.__history = []

    @property
    def mean(self):
        """
        Gets or sets the mean value for the noisy current
        """
        return self.__mean

    @mean.setter
    def mean(self, value):
        """
        Sets the mean value for the noisy current to the given value

        :param value: The new mean current
        """
        self.__mean = value
        self.__history.append(value)

    @property
    def history(self):
        """
        Lists the amplitudes assigned to this device

        :return: A list of float values
        """
        return self.__history
