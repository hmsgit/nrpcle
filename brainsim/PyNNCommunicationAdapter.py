from .BrainInterface import NeuronReference, IBrainCommunicationAdapter, ISpikeGenerator, ISpikeDetector

__author__ = 'GeorgHinkel'


class PyNNCommunicationAdapter(IBrainCommunicationAdapter):
    def initialize(self):
        pass

    def create_spike_generator(self, neurons, spike_generator_type):
        #TODO: implement
        pass

    def create_spike_detector(self, neurons, spike_detector_type):
        #TODO: implement
        pass

    def receive_spikes(self, spikes):
        pass