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
from lazyarray import larray
import numpy as np
import logging

__author__ = 'GeorgHinkel, Igor Peric, Alina Roitberg, Sebastian Krach'
logger = logging.getLogger(__name__)


class PyNNSpikeRecorder(AbstractBrainDevice, ISpikeRecorder):
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
        super(PyNNSpikeRecorder, self).__init__(**params)
        self._spikes = np.array([[], []])
        self._neurons = None
        self._refresh_count = 0

    @property
    def spiked(self):
        """
        Returns the recorded spikes
        "True": neuron spiked within the last time step
        "False": neuron was silent within the last time step
        """
        for spike in self._spikes:
            try:
                if self._neurons.id_to_index(int(spike[0])) is not None:
                    return True
            except IndexError:
                pass
        return False

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
        self._neurons.recorder.reset()
        self._neurons.record('spikes', to_file=False)

    def _stop_record_spikes(self):
        """
        Stops recording the spikes of "neurons"
        """
        self._neurons.record(None)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._neurons = neurons
        self._start_record_spikes()

    def _disconnect(self):
        """
        Stops recording spikes from neurons. This device cannot be used to record again
        after this call.
        """
        if self._neurons:
            self._stop_record_spikes()
            self._neurons = None

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
        return self._neurons.size

    # TODO: this method should be removed since it is not possible to ensure that a device can
    #  provide a meaningful reference to the population is wrapping. A deeper population referencing
    #  refactor is needed.
    @property
    def population_name(self):
        """
        Returns the population name
        """
        return str(self._neurons.parent.label)

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    def refresh(self, time):
        """
        Refreshes the voltage value. The current implementation is VERY slow as PyNN 0.8 wrapps
        the spike data in separate object and in this method we unwrap everything again to be
        compatible with the representation of this device.

        :param time: The current simulation time
        """

        neurons = self._neurons
        block = neurons.get_data("spikes", clear=True)
        segment = block.segments[-1]

        length = 0
        for spiketrain in segment.spiketrains:
            length += len(spiketrain)

        spikedata = np.empty([2, length])

        begin = 0
        end = 0
        for spiketrain in segment.spiketrains:
            end += len(spiketrain)
            if begin == end:
                continue

            neuron_id = larray(spiketrain.annotations['source_index'])
            #neuron_id.shape([end - begin, ])

            spikedata[0][begin:end] = neuron_id.evaluate()
            #spikedata[1][begin:end] = spikedata

        self._spikes = spikedata.T
