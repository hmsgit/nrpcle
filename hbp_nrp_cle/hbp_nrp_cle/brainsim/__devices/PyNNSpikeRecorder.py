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
        self.__spike_count = None
        self.__previous_spike_count = None
        self.__neurons = None

    @property
    def spiked(self):
        """
        Returns the recorded spikes
        "1": neuron spiked within the last time step
        "0": neuron was silent within the last time step
        """
        return self.__spike_count > self.__previous_spike_count

    def start_record_spikes(self):
        """
        Records the spikes of "neurons"

        :param neurons: Population, PopulationView or Assembly
        """
        self.__neurons.record()

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__neurons = neurons
        self.__spike_count = np.zeros(len(self.__neurons))
        self.start_record_spikes()

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        self.__previous_spike_count = self.__spike_count
        self.__spike_count = np.array(
            self.__neurons.get_spike_counts().values())
