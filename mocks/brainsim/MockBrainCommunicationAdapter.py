__author__ = 'GeorgHinkel'

from brainsim.BrainInterface import IBrainCommunicationAdapter, ISpikeGenerator, ISpikeDetector


class MockBrainCommunicationAdapter(IBrainCommunicationAdapter):

    def initialize(self):
        pass

    def register_consume_spikes(self, neurons, spike_detector_type):
        # TODO: implement
        pass

    def register_generate_spikes(self, neurons, spike_generator_type):
        # TODO: implement
        pass

    def refresh_buffers(self):
        pass