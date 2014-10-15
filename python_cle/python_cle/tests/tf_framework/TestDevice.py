"""
This module contains a test device that is used to test the functionality of custom devices
"""

__author__ = 'GeorgHinkel'

from python_cle.brainsim.BrainInterface import ICustomDevice, ILeakyIntegratorAlpha, IACSource, \
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