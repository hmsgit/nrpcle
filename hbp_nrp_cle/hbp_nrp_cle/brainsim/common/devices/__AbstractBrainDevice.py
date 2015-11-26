"""
This module contains an abstract implementation of the IBrainDevice interface and provides
default implementations.
"""
__author__ = "Sebastian Krach"

from .__DeviceGroup import DeviceGroup
from hbp_nrp_cle.brainsim.BrainInterface import IBrainDevice


# PyLint does not recognize the following class as abstract and therefore complains about missing
# implementations
# pylint: disable=abstract-method

class AbstractBrainDevice(IBrainDevice):
    """
    This class provides basic, shared functionality which are common for all brain device
    implementations.
    """

    @classmethod
    def create_new_device(cls, **params):
        """
        Returns a new instance of the concrete implementation of the brain device.

        :param params: additional parameters which are passed to the device constructor
        :return: a new instance of the concrete device
        """
        return cls(**params)

    @classmethod
    def create_new_device_group(cls, length, params):
        """
        Returns a new device group instance for the concrete implementation of this brain device.

        :param length: the size of the group (the amount of nested devices)
        :param params: additional parameters which are passed to the constructor of the nested
            devices. For each parameter either the value can be supplied, or a list of values,
            one for each nested device.
        :return: a new device group containing the created nested devices
        """
        return DeviceGroup.create_new_device_group(cls, length, params)

    def reset(self, transfer_function_manager):
        """
        Resets the device

        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
        """
        return self
