'''
Implementation of PyNNSpikeDetector
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import ISpikeRecorder
import numpy as np
import nest
import logging

__author__ = 'DimitriProbst'
logger = logging.getLogger(__name__)


class PyNNSpikeRecorder(ISpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    # pylint: disable=W0221
    def __init__(self):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        self.__spikes = np.array([[], []])
        self.__previous_spike_count = 0
        self.__neurons = None
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
        # Even though to_file is set to False, an issue in PyNN prevent it to be applied.
        # PyNN is built in such a way that for the spikes in NEST, only the file storage
        # is available. This should be investigated.
        # Meanwhile, we are interacting with NEST directly.
        self.__neurons.record(to_file=False)
        nest.SetStatus(self.__neurons.recorders['spikes']._device.device, "to_memory", True)
        nest.SetStatus(self.__neurons.recorders['spikes']._device.device, "to_file", False)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__neurons = neurons
        self.start_record_spikes()

    # simulation time not necessary for this device
    # pylint: disable=W0613
    # pylint: disable=protected-access
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        # Get the spikes directly from NEST (It let use use memory instead of files)
        self.__spikes = np.array([
            nest.GetStatus(
                self.__neurons.recorders['spikes']._device.device, 'events')
                [0]['senders'][self.__previous_spike_count:],
            nest.GetStatus(
                self.__neurons.recorders['spikes']._device.device, 'events')
                [0]['times'][self.__previous_spike_count:]]).T
        self.__previous_spike_count = nest.GetStatus(
            self.__neurons.recorders['spikes']._device.device, 'n_events')[0] - 1
