'''
Implementation of MockDCSource
moduleauthor: Michael.Weber@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import IDCSource

__author__ = 'MichaelWeber'


class MockDCSource(IDCSource):
    """
    Represents a direct current generator
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
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self.__amplitude

    @amplitude.setter
    def amplitude(self, amplitude):
        """
        Sets the amplitude of the current
        :param amplitude: float
        """
        self.__amplitude = amplitude
        self.__history.append(amplitude)

    @property
    def history(self):
        """
        Lists the amplitudes assigned to this device
        :return: A list of float values
        """
        return self.__history
