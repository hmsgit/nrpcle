"""
The AbstractCommunicationAdapter contains shared code for the communication logic among multiple
implementations of the brain simulator adapter.
"""

import logging
from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter, ICustomDevice

logger = logging.getLogger(__name__)

__author__ = 'Dimitri Probst, Sebastian Krach'


class AbstractCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """

    def __init__(self):
        """
        Initializes the communication adapter
        """
        self.__generator_devices = []
        self.__detector_devices = []
        self.__refreshable_devices = []
        self.__finalizable_devices = []
        self.__is_initialized = False

    def _get_device_type(self, device_type):
        """
        Method that allows implementing classes to define supported device types. The
        implementing class has to return the concrete type for the device interface passed as a
        parameter.

        :param device_type: The device for which the concrete type should be retrieved
        :return: the concrete type
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self):
        """
        Marks the adapter as initialized
        """
        self.__is_initialized = True

    def __register_device(self, populations, device_type, **params):
        """
        Requests a communication object with the given device type for the given set of neurons.

        :param populations: A reference to the populations to which the spike generator
         should be connected
        :param device_type: A brainsim device type
        :param params: A dictionary of configuration parameters
        :return: A communication object or a group of objects
        """
        if isinstance(device_type, ICustomDevice):
            device_type.apply(populations, self, **params)
            return populations

        concrete_type = self._get_device_type(device_type)
        if not isinstance(populations, list):
            device = concrete_type.create_new_device(**params)
            logger.info("Communication object with type \"%s\" requested (device)",
                        device_type)
        else:
            device = concrete_type.create_new_device_group(len(populations), params)
            logger.info("Communication object with type \"%s\" requested (device group)",
                        device_type)
        device.connect(populations)
        return device

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
        device = self.__register_device(populations, spike_generator_type, **params)
        self.__generator_devices.append(device)
        return device

    def unregister_spike_source(self, device):
        """
        Disconnects and unregisters the given spike generator device.

        :param device: The spike generator device to deregister.
        """
        device._disconnect() # pylint: disable=protected-access
        try:
            self.__generator_devices.remove(device)
        except ValueError:
            logger.warn('Attempting to unregister untracked generator device.')

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
        device = self.__register_device(populations, spike_detector_type, **params)
        self.__detector_devices.append(device)
        if hasattr(device, "refresh"):
            self.__refreshable_devices.append(device)
        if hasattr(device, "finalize_refresh"):
            self.__finalizable_devices.append(device)
        return device

    def unregister_spike_sink(self, device):
        """
        Disconnects and unregisters the given spike detector device.

        :param device: The spike detector device to deregister.
        """
        device._disconnect() # pylint: disable=protected-access
        try:
            self.__detector_devices.remove(device)
        except ValueError:
            logger.warn('Attempting to unregister untracked detector device.')

        # optional registractions based on device type
        try:
            self.__refreshable_devices.remove(device)
        except ValueError:
            pass
        try:
            self.__finalizable_devices.remove(device)
        except ValueError:
            pass

    def refresh_buffers(self, t):
        """
        Refreshes all detector buffers

        :param t: The simulation time in milliseconds
        """
        for detector in self.__refreshable_devices:
            detector.refresh(t)
        for detector in self.__finalizable_devices:
            detector.finalize_refresh(t)

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

    @property
    def refreshable_devices(self):
        """
        Gets the created refreshable __devices
        """
        return self.__refreshable_devices

    @property
    def finalizable_devices(self):
        """
        Gets the created finalizable __devices
        """
        return self.__finalizable_devices

    def shutdown(self):
        """
        Shuts down the brain communication adapter
        """
        del self.__detector_devices[:]
        del self.__generator_devices[:]
        del self.__refreshable_devices[:]
        del self.__finalizable_devices[:]
        self.__is_initialized = False
