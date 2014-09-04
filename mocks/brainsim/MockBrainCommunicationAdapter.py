__author__ = 'GeorgHinkel'

from brainsim.BrainInterface import IBrainCommunicationAdapter, NeuronReference, ISpikeGenerator, ISpikeDetector


class MockBrainCommunicationAdapter(IBrainCommunicationAdapter):

    def initialize(self, name):
        pass

    def create_spike_detector(self, neurons, spike_detector_type):
        # TODO: implement
        pass

    def create_spike_generator(self, neurons, spike_generator_type):
        # TODO: implement
        pass

    def refresh_buffers(self):
        pass