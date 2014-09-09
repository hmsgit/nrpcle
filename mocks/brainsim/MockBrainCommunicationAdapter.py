__author__ = 'GeorgHinkel'

from brainsim.BrainInterface import IBrainCommunicationAdapter, INeuronVoltmeter, ICustomDevice

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

        if isinstance(spike_detector_type, ICustomDevice):
            spike_detector_type.apply(neurons, self)
            return spike_detector_type

        # TODO: implement
        meter = MockVoltmeter()
        if isinstance(neurons, list) and hasattr(neurons[0], '__iter__'):
            return list(itertools.repeat(meter, len(neurons)))
        else:
            return meter

    def register_generate_spikes(self, neurons, spike_generator_type):

        if isinstance(spike_generator_type, ICustomDevice):
            spike_generator_type.apply(neurons, self)
            return spike_generator_type

        # TODO: implement
        pass

    def refresh_buffers(self, t):
        pass