'''
Implementation of PyNNSpikeDetector
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import ISpikeDetector
import numpy as np

__author__ = 'DimitriProbst'


class PyNNSpikeDetector(ISpikeDetector):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    def __init__(self, params):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        self.__spike_count = None
        self.__previous_spike_count = None
        self.__neurons = None
        self.__params = params
        self.latest_spikes = None

    def __get_spikes(self):
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

    def refresh(self):
        '''
        Refreshes the spikes record
        '''
        self.latest_spikes = self.__get_spikes()
        return self.latest_spikes
