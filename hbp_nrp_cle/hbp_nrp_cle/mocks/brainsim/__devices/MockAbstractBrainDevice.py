"""
This module contains the shared funtionality among multiple mock brain device implementations.
"""

__author__ = "Sebastian Krach"

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice, DeviceGroup


class MockDeviceGroup(DeviceGroup):
    """
    The class adds functionality to the DeviceGroup implementation to provide future values for
    the nested mock devices
    """

    @classmethod
    def _create_device_config(cls, params, index):
        """
        Creates the configuration for the device with the given index

        :param params: The original parameters
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
        print "mock create device config: %s" % new_params
        return new_params


class AbstractMockBrainDevice(AbstractBrainDevice):
    """
    Shared abstract super class for all mock devices which implements the unused methods to
    reduce the mock implementation file size.
    """

    def reset(self, transfer_function_manager):
        pass

    def connect(self, neurons, **params):
        pass

    @classmethod
    def create_new_device_group(cls, length, params):
        return MockDeviceGroup.create_new_device_group(cls, length, params)
