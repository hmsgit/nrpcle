__author__ = 'GeorgHinkel'

from brainsim.BrainInterface import IBrainCommunicationAdapter, INeuronVoltmeter, ICustomDevice, IPatternSpikeGenerator, \
    IPoissonSpikeGenerator, IFixedFrequencySpikeGenerator, ISpikeRecorder

import itertools


class MockVoltmeter(INeuronVoltmeter):
    def __init__(self):
        self.__voltage = 1

    def __get_voltage(self):
        return self.__voltage

    def __set_voltage(self, value):
        self.__voltage = value

    voltage = property(__get_voltage, __set_voltage)


class MockPatternGenerator(IPatternSpikeGenerator):
    pass


class MockPoissonGenerator(IPoissonSpikeGenerator):
    pass


class MockFixedFrequencyGenerator(IFixedFrequencySpikeGenerator):
    pass


class MockRecorder(ISpikeRecorder):
    pass


class MockBrainCommunicationAdapter(IBrainCommunicationAdapter):
    def __init__(self):
        self.__device_dict = {INeuronVoltmeter: MockVoltmeter, IPatternSpikeGenerator: MockPatternGenerator,
                              IPoissonSpikeGenerator: MockPoissonGenerator,
                              IFixedFrequencySpikeGenerator: MockFixedFrequencyGenerator, ISpikeRecorder: MockRecorder}

    def initialize(self):
        pass

    def register_consume_spikes(self, neurons, spike_detector_type, **config):

        if isinstance(spike_detector_type, ICustomDevice):
            spike_detector_type.apply(neurons, self)
            return spike_detector_type

        device = self.__device_dict[spike_detector_type]()

        if isinstance(neurons, list) and hasattr(neurons[0], '__iter__'):
            return list(itertools.repeat(device, len(neurons)))
        else:
            return device

    def register_generate_spikes(self, neurons, spike_generator_type, **config):

        if isinstance(spike_generator_type, ICustomDevice):
            spike_generator_type.apply(neurons, self)
            return spike_generator_type

        device = self.__device_dict[spike_generator_type]()

        if isinstance(neurons, list) and hasattr(neurons[0], '__iter__'):
            return list(itertools.repeat(device, len(neurons)))
        else:
            return device

    def refresh_buffers(self, t):
        pass