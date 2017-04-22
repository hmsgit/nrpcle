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
'''
This module contains the mock implementations for a spike recorder device which can be set up to
provide predefined values at certain points in time. The device can be used as mock input for
transfer functions, dispensing with the need to run a brain simulation to test them.
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
import warnings
import numpy

__author__ = 'Michael Weber, Georg Hinkel'


class MockSpikeRecorder(AbstractMockBrainDevice, ISpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    default_parameters = {
        "updates": []
    }

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to "infinity"
        is set to infinity by default in order to forbid the neuron to elicit
        spikes.

        :param params: Dictionary of neuron configuration parameters
        """
        super(MockSpikeRecorder, self).__init__(**params)

        self.__spiked = False
        self.__update = self._parameters["updates"]

    @property
    def spiked(self):
        """
        Returns the membrane voltage of the cell
        """
        return self.__spiked

    @property
    def times(self):
        """
        Returns the times and neuron IDs of the recorded spikes within the last time step.
        """
        if self.__spiked:
            return numpy.array([[0.0], [1]])
        else:
            return numpy.array([[], []])

    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        self.__spiked = False
        if hasattr(self.__update, '__getitem__'):
            while len(self.__update) > 0 and isinstance(self.__update[0], float)\
                    and time >= self.__update[0]:
                self.__spiked = True
                self.__update = self.__update[1:]
        else:
            warnings.warn("Updates schedules must be sorted lists of tuples")

    @property
    def updates(self):
        """
        Gets the scheduled updates for this device

        :return: A list of tuples when the mock device should have spiked
        """
        return self.__update

    @updates.setter
    def updates(self, updates):
        """
        Sets the scheduled updates for this device

        :param updates: A new list of update information. This list must consist of times when the
         spike recorder should appear to have spiked
        """
        self.__update = updates
        if updates is None:
            self.__update = []
