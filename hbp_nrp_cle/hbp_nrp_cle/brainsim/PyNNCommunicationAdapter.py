'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

from .BrainInterface import IBrainCommunicationAdapter, \
    ILeakyIntegratorAlpha, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, ISpikeDetector, \
    ILeakyIntegratorExp, IPopulationRate, IFixedSpikeGenerator, ISpikeRecorder

from .__devices.PyNNPoissonSpikeGenerator import PyNNPoissonSpikeGenerator
from .__devices.PyNNFixedSpikeGenerator import PyNNFixedSpikeGenerator
from .__devices.PyNNDCSource import PyNNDCSource
from .__devices.PyNNACSource import PyNNACSource
from .__devices.PyNNNCSource import PyNNNCSource
from .__devices.PyNNLeakyIntegratorAlpha import PyNNLeakyIntegratorAlpha
from .__devices.PyNNLeakyIntegratorExp import PyNNLeakyIntegratorExp
from .__devices.PyNNPopulationRate import PyNNPopulationRate
from .__devices.PyNNSpikeRecorder import PyNNSpikeRecorder
from .__devices.PyNNDeviceGroup import PyNNDeviceGroup
import logging

logger = logging.getLogger(__name__)

__author__ = 'DimitriProbst'


class PyNNCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IFixedSpikeGenerator: PyNNFixedSpikeGenerator,
                     IPoissonSpikeGenerator: PyNNPoissonSpikeGenerator,
                     IDCSource: PyNNDCSource,
                     IACSource: PyNNACSource,
                     INCSource: PyNNNCSource,
                     ILeakyIntegratorAlpha: PyNNLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: PyNNLeakyIntegratorExp,
                     IPopulationRate: PyNNPopulationRate,
                     ISpikeRecorder: PyNNSpikeRecorder,
                     ISpikeDetector: PyNNSpikeRecorder}

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
        logger.info("PyNN communication adapter initialized")

    def register_spike_source(self, populations, spike_generator_type, **params):
        """
        Requests a communication object with the given spike generator type
        for the given set of neurons

        :param populations: A reference to the populations to which the spike generator
         should be connected
        :param spike_generator_type: A spike generator type (see documentation
         or a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A communication object or a group of objects
        """
        if not isinstance(populations, list):
            device = PyNNCommunicationAdapter.__device_dict[
                spike_generator_type](params)
            device.connect(populations, params)
            self.__generator_devices.append(device)
            logger.info("Communication object with spike generator\
 type \"%s\" requested (device)",
                        spike_generator_type)
            return device
        else:
            device_list = []
            device_type = PyNNCommunicationAdapter.__device_dict[
                spike_generator_type]
            for (pop_index, pop) in enumerate(populations):
                device = device_type(
                    PyNNCommunicationAdapter.__create_device_config(params, pop_index))
                device.connect(pop, params)
                device_list.append(device)
            self.__generator_devices += device_list
            device_group = PyNNDeviceGroup(device_list)
            logger.info("Communication object with spike generator\
 type \"%s\" requested (device group)",
                        spike_generator_type)
            return device_group

    def register_spike_sink(self, populations, spike_detector_type, **params):
        """
        Requests a communication object with the given spike detector type
        for the given set of neurons

        :param populations: A reference to the populations which should be connected
         to the spike detector
        :param spike_detector_type: A spike detector type (see documentation
         for a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A Communication object or a group of objects
        """
        if not isinstance(populations, list):
            device = PyNNCommunicationAdapter.__device_dict[
                spike_detector_type](params)
            device.connect(populations, params)
            self.__detector_devices.append(device)
            logger.info("Communication object with spike detector\
 type \"%s\" requested (device)",
                        spike_detector_type)
            return device
        else:
            device_list = []
            device_type = PyNNCommunicationAdapter.__device_dict[
                spike_detector_type]
            for (pop_index, pop) in enumerate(populations):
                device = device_type(
                    PyNNCommunicationAdapter.__create_device_config(params, pop_index))
                device.connect(pop, params)
                device_list.append(device)
            self.__detector_devices += device_list
            device_group = PyNNDeviceGroup(device_list)
            logger.info("Communication object with spike detector\
 type \"%s\" requested (device group)",
                        spike_detector_type)
            return device_group

    def refresh_buffers(self, t):
        """
        Refreshes all detector buffers

        :param t: The simulation time in milliseconds
        """
        for detector in self.__detector_devices:
            if hasattr(detector, "refresh"):
                detector.refresh(t)

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
            if hasattr(value, '__getitem__'):
                value = value[index]
            new_params[key] = value
        return new_params

    @property
    def detector_devices(self):
        """
        Gets the created detector __devices
        """
        return self.__detector_devices

    @property
    def generator_devices(self):
        """
        Gets the created spike detector __devices
        """
        return self.__generator_devices

    @property
    def is_initialized(self):
        """
        Gets a value indicating whether initialize has been called
        """
        return self.__is_initialized
