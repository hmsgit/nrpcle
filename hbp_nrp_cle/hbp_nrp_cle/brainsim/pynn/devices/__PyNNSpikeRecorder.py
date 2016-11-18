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

        self._neurons.record('spikes', to_file=False)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._neurons = neurons
        self._start_record_spikes()

    @property
    def neurons(self):
        """
        Gets the neurons monitored by this spike recorder
        """
        return self._neurons

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
            neuron_id.shape([end - begin, ])

            spikedata[0][begin:end] = neuron_id.evaluate()
            spikedata[1][begin:end] = spikedata

        self._spikes = spikedata.T
