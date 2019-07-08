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
Implementation of PyNNSpiNNakerSpikeRecorder
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connections
import numpy as np
import logging

__author__ = 'Georg Hinkel'
logger = logging.getLogger(__name__)


class PyNNSpiNNakerSpikeRecorder(AbstractBrainDevice, ISpikeRecorder):  # pragma no cover
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    default_parameters = {
        "use_ids": False
    }

    def __init__(self, **params):
        """
        Creates a new live spike recorder
        """
        super(PyNNSpiNNakerSpikeRecorder, self).__init__(**params)
        if params.get("use_ids"):
            raise Exception("Usage of ids is currently not supported in SpiNNaker")
        self.__connection = None
        self.__time = 0
        self.__neurons = None
        self.__spike_neurons = []
        self.__spike_times = []

    @property
    def times(self):
        """
        Returns the times and neuron IDs of the recorded spikes within the last time step.
        """
        spikes = []
        times = []

        try:
            for i, spikes_at_i in enumerate(self.__spike_neurons):
                spikes += spikes_at_i
                times += [self.__spike_times[i]] * len(spikes_at_i)
        # it may happen that the spikes are being cleared from a different thread
        except IndexError:  # pragma: no cover
            pass
        return np.array([spikes, times]).T

    @property
    def spiked(self):
        """
        Gets a value indicating whether the neuron has spiked since the last iteration
        """
        return len(self.__spike_times) > 0

    # pylint: disable=unused-argument
    def __receive_spike(self, label, time, neuron_ids):
        """
        Handles that a new spike is recorded

        :param label: The recorded label (ignored)
        :param time: The simulation time
        :param neuron_ids: The neurons that have spiked
        """
        logger.debug("Received spikes from {ids} at time {t}".format(ids=neuron_ids, t=self.__time))
        self.__spike_times.append(self.__time)
        self.__spike_neurons.append(list(neuron_ids))

    def _disconnect(self):
        pass

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__neurons = neurons
        live_connections.register_receiver(neurons, self.__receive_spike)

    @property
    def neurons(self):
        """
        Returns the neurons
        """
        return self.__neurons

    @property
    def neurons_count(self):
        """
        Returns the neurons count
        """
        return self.__neurons.size

    @property
    def population_name(self):
        """
        Returns the population name
        """
        return str(self.__neurons.label)

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    def refresh(self, time):
        """
        Refreshes the voltage value.

        :param time: The current simulation time
        """
        # Spikes are recorded on the fly
        self.__time = time

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    # pylint: disable=protected-access
    def finalize_refresh(self, time):
        """
        Resets the number of spikes for the connected spike recorder, this is a PyNN-Nest specific
        command that clears all memory used by a recorder.

        :param time: The current simulation time
        """
        del self.__spike_neurons[:]
        del self.__spike_times[:]
        self.__time = time
