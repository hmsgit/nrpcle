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
Implementation of PyNNSpikeDetector
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
import numpy as np
import logging

__author__ = 'Felix Schneider'
logger = logging.getLogger(__name__)


class PyNNSpiNNakerSpikeRecorder(AbstractBrainDevice, ISpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    # No connection parameters necessary for this device
    # pylint: disable=W0613
    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        super(PyNNSpiNNakerSpikeRecorder, self).__init__(**params)
        self.__spikes = np.array([[], []])
        self._neurons = None
        self.__refresh_count = 0

    @property
    def spiked(self):
        """
        Returns the recorded spikes
        "1": neuron spiked within the last time step
        "0": neuron was silent within the last time step
        """
        return self.__spikes.shape[0] > 0

    @property
    def times(self):
        """
        Returns the times and neuron IDs of the recorded spikes within the last time step.
        :return:
        """
        return self.__spikes

    # pylint: disable=protected-access
    def start_record_spikes(self):
        """
        Records the spikes of "neurons"
        """
        print "Record spike recorder"
        self._neurons.record(to_file=None)

    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._neurons = neurons
        self.start_record_spikes()

    def _disconnect(self):
        """
        INTERNAL USE ONLY: this should never be directly invoked by a user.

        Disconnects the brain device from any output neuron populations. This device will no longer
        interact with the brain after disconnect is called.
        """
        if self._neurons is not None:
            self._neurons = None

    # simulation time not necessary for this device
    # pylint: disable=W0613
    # pylint: disable=protected-access
    def refresh(self, time):
        """
        Refreshes the voltage value. The current implementation is VERY slow as PyNN 0.8 wrapps
        the spike data in separate object and in this method we unwrap everything again to be
        compatible with the representation of this device.

        :param time: The current simulation time
        """
        raise NotImplementedError()
