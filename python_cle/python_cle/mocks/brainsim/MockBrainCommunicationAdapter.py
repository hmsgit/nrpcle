'''
MockBrainCommunicationAdapter.py
moduleauthor: Michael.Weber@fzi.de
'''


from python_cle.brainsim.BrainInterface import IBrainCommunicationAdapter, IIFCurrAlpha, \
    ISpikeDetector, IPoissonSpikeGenerator, IDCSource, IACSource, INCSource
from .devices.MockPoissonSpikeGenerator import MockPoissonSpikeGenerator
from .devices.MockDCSource import MockDCSource
from .devices.MockACSource import MockACSource
from .devices.MockNCSource import MockNCSource
from .devices.MockIFCurrAlpha import MockIFCurrAlpha
from .devices.MockSpikeDetector import MockSpikeDetector

__author__ = 'MichaelWeber'


class MockBrainCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents a mock of the brain communication adapter
    """
    #pylint: disable=W0223
    #pylint: disable=W0613
    # In this dictionary, the association of spike generator types to classes implementing their
    # functionality is established
    __device_dict = {IPoissonSpikeGenerator: MockPoissonSpikeGenerator,
                     IDCSource: MockDCSource,
                     IACSource: MockACSource,
                     INCSource: MockNCSource,
                     IIFCurrAlpha: MockIFCurrAlpha,
                     ISpikeDetector: MockSpikeDetector}

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
        device = MockBrainCommunicationAdapter.__device_dict[
            spike_generator_type](**params)
        device.connect(neurons, **params)
        self.__generator_devices.append(device)
        return device

    def register_spike_sink(self, neurons, spike_detector_type, **params):
        '''
        Requests a communication object with the given spike detector type
        for the given set of neurons
        :param neurons: A reference to the neurons which should be connected
        to the spike detector
        :param spike_detector_type: A spike detector type (see documentation
        for a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A Communication object
        '''
        device = MockBrainCommunicationAdapter.__device_dict[
            spike_detector_type](**params)
        device.connect(neurons, **params)
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
        Gets the detector devices created by this mock
        """
        return self.__detector_devices

    @property
    def generator_devices(self):
        """
        Gets the spike detector devices created by this mock
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
