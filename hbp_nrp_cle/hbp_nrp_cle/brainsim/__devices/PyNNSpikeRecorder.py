'''
Implementation of PyNNSpikeDetector
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import ISpikeRecorder
import numpy as np

__author__ = 'DimitriProbst'


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

    @property
    def spiked(self):
        """
        Returns the recorded spikes
        "1": neuron spiked within the last time step
        "0": neuron was silent within the last time step
        """
        return self.__spikes.shape[0] > self.__previous_spike_count

    @property
    def times(self):
        """
        Returns the times and neuron IDs of the recorded spikes within the last time step.
        :return:
        """
        return self.__spikes[self.__previous_spike_count:, :]

    def start_record_spikes(self):
        """
        Records the spikes of "neurons"
        """
        self.__neurons.record('spikes')

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
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        self.__previous_spike_count = self.__spikes.shape[0]
        self.__spikes = self.__neurons.getSpikes()
