'''
Implementation of DeviceGroup
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import IDeviceGroup
import numpy

__author__ = 'DimitriProbst, Sebastian Krach'


class DeviceGroup(IDeviceGroup):
    """
    Gathers multiple devices of the same type to a group.
    The class receives the property values as slice, tuple, list, numpy array
    and returns the property values as numpy array.
    """

    def __init__(self, devices):
        """
        Initializes a device group
        """
        # use this form since __setattr__ is overridden
        self.__dict__['devices'] = devices

    @classmethod
    def create_new_device_group(cls, nested_device_type, length, params):
        """
        Creates a new device group of the specified size consisting of the brainsim devices of
        the specified type.

        :param nested_device_type: The concrete type of brain device to instantiate
        :param length: the amount of devices to create
        :param params: additional parameters which are passed to the constructor of the nested
            devices. For each parameter either the value can be supplied, or a list of values,
            one for each nested device.
        :return: a new device group
        """
        devices = list()
        for i in range(0, length):
            device = nested_device_type.create_new_device(
                **cls._create_device_config(params, i))
            devices.append(device)
        return cls(devices)

    def __getitem__(self, index):
        if isinstance(index, (slice, numpy.ndarray)):
            return DeviceGroup(self.devices[index])
        elif isinstance(index, int):
            return self.devices[index]
        else:
            raise TypeError

    def __len__(self):
        return len(self.devices)

    def __getattr__(self, attrname):
        if hasattr(DeviceGroup, attrname):
            return super(DeviceGroup, self).__getattr__(attrname)
        array = numpy.zeros(len(self.devices))
        i = 0
        for device in self.devices:
            array[i] = getattr(device, attrname)
            i += 1
        return array

    def __setattr__(self, attrname, value):
        if attrname in self.__dict__:
            self.__dict__[attrname] = value
        else:
            if hasattr(value, '__getitem__'):
                i = 0
                for device in self.devices:
                    setattr(device, attrname, value[i])
                    i += 1
            else:
                for device in self.devices:
                    setattr(device, attrname, value)

    def refresh(self, t):
        """
        Refreshes all inner devices at time t

        :param t: The current simulation time
        """
        for device in self.devices:
            if hasattr(device, 'refresh'):
                device.refresh(t)

    def connect(self, neurons, **params):
        """
        Connects the contained devices to the neurons specified as parameter.

        :param neurons: the neuron population
        :param params: additional parameters for the connection
        """
        for pop_index, (pop, dev) in enumerate(zip(neurons, self.devices)):
            dev.connect(pop, **self._create_device_config(params, pop_index))

    def reset(self, transfer_function_manager):
        """
        Resets the nested devices

        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset device group
        """
        reset_devices = list()
        for device in self.devices:
            reset_devices.append(device.reset(transfer_function_manager))
        return DeviceGroup(reset_devices)

    @classmethod
    def _create_device_config(cls, params, index):
        """
        Creates the configuration for the device with the given index

        :param params: The original parameters
        :param index: The index of the device
        :return: A parameter array
        """
        new_params = {}
        for key in params:
            value = params[key]
            if isinstance(value, list) or isinstance(value, numpy.ndarray):
                value = value[index]
            new_params[key] = value
        return new_params
