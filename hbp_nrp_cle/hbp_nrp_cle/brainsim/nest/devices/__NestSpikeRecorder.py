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
Implementation of NestSpikeDetector
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
import numpy as np

import nest

__author__ = 'LorenzoVannucci'


class NestSpikeRecorder(AbstractBrainDevice, ISpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    default_parameters = {
        "use_ids": True
    }

    # No connection parameters necessary for this device
    # pylint: disable=W0613
    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        super(NestSpikeRecorder, self).__init__(**params)
        self._spikes = np.array([[], []])
        self._recorder = None
        self._neurons = None
        self._refresh_count = 0
        self.__use_ids = self.get_parameters().get("use_ids")
        self.create_device()

    @property
    def spiked(self):
        """
        Checks whether a spike has occurred
        "True": neuron spiked within the last time step
        "False": neuron was silent within the last time step
        """
        return len(self._spikes) > 0

    @property
    def times(self):
        """
        Returns a two dimensional numpy array containing the times and the IDs of the recorded
        spikes within the last time step. The structure is as follows:

        |NeuronID|time_stamp|

        The array is sorted in ascending order of the time stamps.

        :return: a numpy array containing times and neuron IDs of neurons which spiked.
        """
        return self._spikes

    def _start_record_spikes(self):
        """
        Records the spikes of "neurons"
        """
        nest.SetStatus(self._recorder, {'n_events': 0, 'frozen': False})

    def _stop_record_spikes(self):
        """
        Stops recording the spikes of "neurons"
        """
        nest.SetStatus(self._recorder, {'n_events': 0, 'frozen': True})

    def create_device(self):
        """
        Create Poisson spike generator device
        """
        self._recorder = nest.Create('spike_detector', 1)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        self._neurons = neurons
        nest.Connect(neurons, self._recorder)

    def _disconnect(self):
        """
        Stops recording spikes from neurons. This device cannot be used to record again
        after this call.
        """
        pass  # nest.Disconnect does not work properly in NEST 2.12

    @property
    def neurons(self):
        """
        Gets the neurons monitored by this spike recorder
        """
        return self._neurons

    @property
    def neurons_count(self):
        """
        Gets the number of neurons monitored by this spike recorder
        """
        return len(self._neurons)

    # pylint: disable=no-self-use
    @property
    def population_name(self):
        """
        Returns the population name
        """
        # In NEST, there is no label associated for populations
        return ""

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    def refresh(self, time):
        """
        Refreshes the recorded spikes

        :param time: The current simulation time
        """
        spikes_nest, times_nest = self.__read_recorder_data()
        self._spikes = np.array([spikes_nest, times_nest]).T
        nest.SetStatus(self._recorder, {'n_events': 0})

    def __read_recorder_data(self):
        """
        Reads the recorded data of the given recorder

        :return: (spikes, times)
        """
        # Get the spikes
        nest_info = nest.GetStatus(self._recorder, 'events')[0]

        times_nest = nest_info['times']
        spikes_nest = nest_info['senders']
        return spikes_nest, times_nest
