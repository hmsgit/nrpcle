'''
Implementation of MockDCSource
moduleauthor: Michael.Weber@fzi.de
'''

from python_cle.brainsim.BrainInterface import IDCSource

__author__ = 'MichaelWeber'


class MockDCSource(IDCSource):
    """
    Represents a direct current generator
    :param kwargs: Optional configuration parameters
    """
    #pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a direct current generator.
        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """
        self.__generator = None
        self.__amplitude = 1.0
        self.create_device(**params)

    @property
    def amplitude(self):
        '''
        Returns the amplitude of the current
        '''
        return self.__amplitude

    @amplitude.setter
    def amplitude(self, value):
        '''
        Sets the amplitude of the current
        :param amplitude: float
        '''
        self.__amplitude = value

    def create_device(self, **params):
        '''
        Create a direct current source
        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        '''

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
