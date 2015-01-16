"""
This module contains a test device that is used to test the functionality of custom devices
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.brainsim.BrainInterface import ICustomDevice, ILeakyIntegratorAlpha, IACSource, \
    IBrainCommunicationAdapter


class TestDevice(ICustomDevice):
    """
    Represents a device for testing purposes
    """

    def __init__(self):
        """
        Creates a new test device
        """
        self.__source = None

    def apply(self, neurons, brain_adapter, **config):
        """
        Binds the test device to the specified set of neurons
        :param neurons: The neurons
        :param brain_adapter: The brain adapter
        :param config: Additional device configuration
        """
        assert isinstance(brain_adapter, IBrainCommunicationAdapter)
        self.__source = brain_adapter.register_spike_source(neurons, IACSource, **config)

    @property
    def inner(self):
        """
        Gets the inner AC source managed by this test device
        """
        return self.__source

    def reset(self, transfer_function_manager):
        """
        Resets the test device
        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
        """
        new_device = TestDevice()
        new_device.__source = self.__source.reset(transfer_function_manager)
        return new_device