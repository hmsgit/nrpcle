'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

from .BrainInterface import IBrainCommunicationAdapter, IIFCurrAlpha, \
    ISpikeDetector, IPoissonSpikeGenerator, IDCSource, IACSource, INCSource
from .devices.PyNNPoissonSpikeGenerator import \
    PyNNPoissonSpikeGenerator
from .devices.PyNNDCSource import PyNNDCSource
from .devices.PyNNACSource import PyNNACSource
from .devices.PyNNNCSource import PyNNNCSource
from .devices.PyNNIFCurrAlpha import PyNNIFCurrAlpha
from .devices.PyNNSpikeDetector import PyNNSpikeDetector

__author__ = 'DimitriProbst'


class PyNNCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IPoissonSpikeGenerator: PyNNPoissonSpikeGenerator,
                     IDCSource: PyNNDCSource,
                     IACSource: PyNNACSource,
                     INCSource: PyNNNCSource,
                     IIFCurrAlpha: PyNNIFCurrAlpha,
                     ISpikeDetector: PyNNSpikeDetector}

    def __init__(self):
        """
        Initializes the communication adapter
        """
        self.__generator_devices = []
        self.__detector_devices = []
        self.__is_initialized = False

    def initialize(self):
        """
        Marks the PyNN adapter as initialized
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
        device = PyNNCommunicationAdapter.__device_dict[
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
        device = PyNNCommunicationAdapter.__device_dict[
            spike_detector_type](**params)
        device.connect(neurons, **params)
        self.__detector_devices.append(device)
        return device

    def refresh_buffers(self, t):
        """
        Refreshes all detector buffers
        """
        for detector in self.__detector_devices:
            if hasattr(detector, "refresh"):
                detector.refresh(t)

    @property
    def detector_devices(self):
        """
        Gets the created detector devices
        """
        return self.__detector_devices

    @property
    def generator_devices(self):
        """
        Gets the created spike detector devices
        """
        return self.__generator_devices

    @property
    def is_initialized(self):
        """
        Gets a value indicating whether initialize has been called
        """
        return self.__is_initialized
