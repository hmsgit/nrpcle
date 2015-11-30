'''
This module contains the mock implementations for a noisy current source which records the
changes to the supported device parameters and can be used to track transfer function activity.
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import INCSource

__author__ = 'MichaelWeber'


class MockNCSource(AbstractMockBrainDevice, INCSource):
    """
    Represents a Mock of a noisy current generator
    """

    default_parameters = {
        "mean": 1.0
    }

    #pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        """
        super(MockNCSource, self).__init__(**params)

        self.__mean = self._parameters["mean"]
        self.__history = []

    @property
    def mean(self):
        """
        Gets or sets the mean value for the noisy current
        """
        return self.__mean

    @mean.setter
    def mean(self, value):
        """
        Sets the mean value for the noisy current to the given value

        :param value: The new mean current
        """
        self.__mean = value
        self.__history.append(value)

    @property
    def history(self):
        """
        Lists the amplitudes assigned to this device

        :return: A list of float values
        """
        return self.__history
