'''
MockBrainCommunicationAdapter.py
moduleauthor: Michael.Weber@fzi.de
'''


from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter, \
    ILeakyIntegratorAlpha, ISpikeDetector, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, ICustomDevice, \
    IFixedSpikeGenerator, ILeakyIntegratorExp, IPopulationRate
from .__devices.MockPoissonSpikeGenerator import MockPoissonSpikeGenerator
from .__devices.MockFixedSpikeGenerator import MockFixedSpikeGenerator
from .__devices.MockDCSource import MockDCSource
from .__devices.MockACSource import MockACSource
from .__devices.MockNCSource import MockNCSource
from .__devices.MockLeakyIntegratorAlpha import MockLeakyIntegratorAlpha
from .__devices.MockLeakyIntegratorExp import MockLeakyIntegratorExp
from .__devices.MockPopulationRate import MockPopulationRate
from .__devices.MockSpikeRecorder import MockSpikeRecorder
from .__devices.MockDeviceGroup import MockDeviceGroup

__author__ = 'MichaelWeber'


class MockBrainCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents a mock of the brain communication adapter
    """
    #pylint: disable=W0223
    #pylint: disable=W0613
    # In this dictionary, the association of spike generator types to classes implementing their
    # functionality is established
    __device_dict = {IFixedSpikeGenerator: MockFixedSpikeGenerator,
                     IPoissonSpikeGenerator: MockPoissonSpikeGenerator,
                     IDCSource: MockDCSource,
                     IACSource: MockACSource,
                     INCSource: MockNCSource,
                     ILeakyIntegratorAlpha: MockLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: MockLeakyIntegratorExp,
                     IPopulationRate: MockPopulationRate,
                     ISpikeDetector: MockSpikeRecorder}

    def __init__(self):
        """
        Initializes a new mocked brain communication adapter
        """
        self.__generator_devices = []
        self.__detector_devices = []
        self.__is_initialized = False
        self.__refreshed_at = []

    def initialize(self):
        """
        Marks the mock adapter as initialized
        """
        self.__is_initialized = True

    def register_spike_source(self, neurons, spike_generator_type, **params):
        """
        Requests a communication object with the given spike generator type
        for the given set of neurons

        :param neurons: A reference to the neurons to which the spike generator
         should be connected
        :param spike_generator_type: A spike generator type (see documentation
         or a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A communication object
        """
        if isinstance(spike_generator_type, ICustomDevice):
            spike_generator_type.apply(neurons, self, **params)
            return spike_generator_type

        if not isinstance(neurons, list):
            device = MockBrainCommunicationAdapter.__device_dict[
                spike_generator_type](**params)
        else:
            device_list = []
            device_type = MockBrainCommunicationAdapter.__device_dict[
                spike_generator_type]
            neuron_index = 0
            for n in neurons:
                new_parameters = MockBrainCommunicationAdapter.__create_device_config(
                    params, neuron_index)
                new_parameters['neurons'] = n
                device_list.append(device_type(**new_parameters))
                neuron_index += 1
            device = MockDeviceGroup(device_list)
        self.__generator_devices.append(device)
        return device

    @staticmethod
    def __create_device_config(params, index):
        """
        Creates the configuration for the device with the given index

        :param params: The original parameters
        :param index: The index of the device
        :return: A parameter array
        """
        new_params = {}
        for key in params:
            value = params[key]
            if key == 'updates':
                new_value = []
                for (t, values) in value:
                    new_value.append((t, values[index]))
                value = new_value
            else:
                if hasattr(value, '__getitem__'):
                    value = value[index]
            new_params[key] = value
        return new_params

    def register_spike_sink(self, neurons, spike_detector_type, **params):
        """
        Requests a communication object with the given spike detector type
        for the given set of neurons

        :param neurons: A reference to the neurons which should be connected
         to the spike detector
        :param spike_detector_type: A spike detector type (see documentation
         for a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A Communication object
        """
        if isinstance(spike_detector_type, ICustomDevice):
            spike_detector_type.apply(neurons, self, **params)
            return spike_detector_type

        if not isinstance(neurons, list):
            device = MockBrainCommunicationAdapter.__device_dict[
                spike_detector_type](**params)
        else:
            device_list = []
            device_type = MockBrainCommunicationAdapter.__device_dict[
                spike_detector_type]
            neuron_index = 0
            for n in neurons:
                new_parameters = MockBrainCommunicationAdapter.__create_device_config(
                    params, neuron_index)
                new_parameters['neurons'] = n
                device_list.append(device_type(**new_parameters))
                neuron_index += 1
            device = MockDeviceGroup(device_list)
        self.__detector_devices.append(device)
        return device

    def refresh_buffers(self, t):
        """
        Refreshes buffered values for time t

        :param t: The brain simulation time
        """
        self.__refreshed_at.append(t)
        for detector in self.__detector_devices:
            if hasattr(detector, "refresh"):
                detector.refresh(t)

    @property
    def detector_devices(self):
        """
        Gets the detector __devices created by this mock
        """
        return self.__detector_devices

    @property
    def generator_devices(self):
        """
        Gets the spike detector __devices created by this mock
        """
        return self.__generator_devices

    @property
    def is_initialized(self):
        """
        Gets a value indicating whether initialize has been called
        """
        return self.__is_initialized

    @property
    def refreshed_at(self):
        """
        Gets a list of simulation times at which the mock has been refreshed
        """
        return self.__refreshed_at

    def shutdown(self):
        """
        Shuts down the brain communication adapter
        """
        pass
