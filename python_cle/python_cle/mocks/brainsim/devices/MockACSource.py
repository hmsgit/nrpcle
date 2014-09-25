'''
Implementation of MockACSource
moduleauthor: Michael.Weber@fzi.de
'''

from python_cle.brainsim.BrainInterface import IACSource

__author__ = 'MichaelWeber'


class MockACSource(IACSource):
    """
    Represents an alternating current generator.
    :param kwargs: Optional configuration parameters
    """
    #pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.
        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        :param offset: Offset of alternating current, default: 0.0 nA
        :param frequency: Frequency of alternating current, default: 10.0 Hz
        :param phase: Phase of alternating current, default: 0.0
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, dafault: infinity
        """
        self.__generator = None
        self.__amplitude = 1.0
        self.__offset = 0.0
        self.__frequency = 10.0
        self.__phase = 0.0

        self.create_device(**params)

    @property
    def amplitude(self):
        '''
        Returns the amplitude of the current
        '''
        return self.__amplitude

    @amplitude.setter
    def amplitude(self, amplitude):
        '''
        Sets the amplitude of the current
        :param amplitude: float
        '''
        self.__amplitude = amplitude

    @property
    def offset(self):
        '''
        Returns the offset of the current
        '''
        return self.__offset

    @offset.setter
    def offset(self, offset):
        '''
        Sets the offset of the current
        :param offset: float
        '''
        self.__offset = offset

    @property
    def frequency(self):
        '''
        Returns the frequency of the current
        '''
        return self.__frequency

    @frequency.setter
    def frequency(self, frequency):
        '''
        Sets the frequency of the current
        :param frequency: float
        '''
        self.__frequency = frequency

    @property
    def phase(self):
        '''
        Returns the phase of the current
        '''
        return self.__phase

    @phase.setter
    def phase(self, phase):
        '''
        Sets the phase of the current
        :param phase: float
        '''
        self.__phase = phase

    def create_device(self, **params):
        '''
        Creates an alternating current source
        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        :param offset: Offset of alternating current, default: 0.0 nA
        :param frequency: Frequency of alternating current, default: 10 Hz
        :param phase: Phase of alternating current, default: 0.0
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, dafault: infinity
        '''

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
