'''
Implementation of PyNNSpikeDetector
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import ISpikeDetector
import numpy as np

__author__ = 'DimitriProbst'


class PyNNSpikeDetector(ISpikeDetector):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    def __init__(self):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        self.__spike_count = None
        self.__previous_spike_count = None
        self.__neurons = None
        self.__update = [0.0, None]

    def __get_spikes(self):
        '''
        Returns the recorded spikes
        "1": neuron spiked within the last time step
        "0": neuron was silent within the last time step
        '''
        self.__previous_spike_count = self.__spike_count
        self.__spike_count = np.array(
            self.__neurons.get_spike_counts().values())
        return self.__spike_count > self.__previous_spike_count

    spikes = property(__get_spikes)

    def start_record_spikes(self):
        '''
        Records the spikes of "neurons"
        :param neurons: Population, PopulationView or Assembly
        '''
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

    def refresh(self, time):
        '''
        Refreshes the voltage value
        :param time: The current simulation time
        '''
        if self.__update[0] is not time:
            self.__update[0] = time
            self.__update[1] = self.__get_spikes()
        return self.__update[1]
