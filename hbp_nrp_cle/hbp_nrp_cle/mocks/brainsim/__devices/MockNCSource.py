'''
Implementation of MockNCSource
moduleauthor: Michael.Weber@fzi.de
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import INCSource

__author__ = 'MichaelWeber'


class MockNCSource(AbstractMockBrainDevice, INCSource):
    """
    Represents ai Mock of a noisy current generator
    """
    #pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        """
        self.__amplitude = params.get('amplitude', 1.0)
        self.__history = []

    @property
    def mean(self):
        """
        Gets or sets the mean value for the noisy current
        """
        return self.__amplitude

    @mean.setter
    def mean(self, value):
        """
        Sets the mean value for the noisy current to the given value

        :param value: The new mean current
        """
        self.__amplitude = value
        self.__history.append(value)

    @property
    def history(self):
        """
        Lists the amplitudes assigned to this device

        :return: A list of float values
        """
        return self.__history
