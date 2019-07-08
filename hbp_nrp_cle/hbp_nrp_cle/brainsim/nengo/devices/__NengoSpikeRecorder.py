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
Implementation of NengoSpikeDetector
'''

from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice

import numpy as np
import nengo


class NengoSpikeRecorder(AbstractBrainDevice, ISpikeRecorder): # pragma: no cover
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    default_parameters = {
        "use_ids": True
    }

    def __init__(self, nengo_simulation_state, **kwargs):

        # Remove key "direction_kind" from kwargs, since this key is not required
        # Leaving this key in kwargs would cause an error in the subsequent parameter verification
        if 'direction_kind' in kwargs:
            del kwargs['direction_kind']

        super(NengoSpikeRecorder, self).__init__(**kwargs)
        self.__nengo_simulation_state = nengo_simulation_state
        self.__spiked = False
        self.__spikes = np.array([[], []])
        self.__probe = None
        self._neurons = None
        self._neurons_count = None

    @property
    def spiked(self):
        return self.__spiked

    @property
    def times(self):
        return self.__spikes

    def connect(self, neurons):

        with self.__nengo_simulation_state:
            self.__probe = nengo.Probe(neurons.neurons)

        self._neurons = neurons

    def reset(self, transfer_function_manager):
        raise NotImplementedError

    def refresh(self, t):
        """
        Refreshes the recorded spikes

        :param t: The current simulation time
        """
        sim = self.__nengo_simulation_state.simulator

        # Get spike data from Nengo simulator
        spikes = sim.data[self.__probe]

        # Calculate number of spikes and initialize Numpy array
        count_spikes = np.count_nonzero(spikes)
        self.__spikes = np.zeros((count_spikes, 2))

        # Initialize time stamp
        time_new = t

        # Index variable for output spike array
        j = 0

        # Read array with spike data line by line
        # One line contains the information for each neuron if it has spiked
        for row in spikes:

            time_new += sim.dt

            for neuron_i, neuron in enumerate(row):

                # Neuron==1000 means that this neuron has spiked, neuron==0 means it has not spiked
                if neuron == 1000:
                    self.__spikes[j, 0] = neuron_i
                    # Convert time from seconds to milliseconds and add to the spike array
                    self.__spikes[j, 1] = 1000 * time_new
                    j += 1

        # Reset cache from ProbeDict data
        sim.data.reset()

        # Clear probe data
        self.__nengo_simulation_state.clear_probe_data(self.__probe)

    def _disconnect(self):
        del self.__probe

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
        return self._neurons.n_neurons

    @property
    def population_name(self):
        """
        Returns the population name
        """
        return str(self._neurons.label)
