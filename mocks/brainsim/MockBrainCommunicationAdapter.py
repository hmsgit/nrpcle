__author__ = 'GeorgHinkel'

from brainsim.BrainInterface import IBrainCommunicationAdapter, INeuronVoltmeter

import itertools

class MockVoltmeter(INeuronVoltmeter):

    def __init__(self):
        self.__voltage = 1

    def __get_voltage(self):
        return self.__voltage

    def __set_voltage(self, value):
        self.__voltage = value

    voltage = property(__get_voltage, __set_voltage)


class MockBrainCommunicationAdapter(IBrainCommunicationAdapter):

    def initialize(self):
        pass

    def register_consume_spikes(self, neurons, spike_detector_type):
        # TODO: implement
        meter = MockVoltmeter()
        if len(neurons) == 1:
            return meter
        else:
            return list(itertools.repeat(meter, len(neurons)))

    def register_generate_spikes(self, neurons, spike_generator_type):
        # TODO: implement
        pass

    def refresh_buffers(self, t):
        pass