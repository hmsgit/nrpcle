'''
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IACSource
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNACSource(IACSource):
    """
    Represents an alternating current generator.
    :param kwargs: Optional configuration parameters
    """

    def __init__(self, **params):
        """
        Initializes an alternating current generator.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__generator = None

        self.create_device(**params)

    def __get_amplitude(self):
        '''
        Returns the amplitude of the current
        '''
        return self.__generator.amplitude

    def __set_amplitude(self, amplitude):
        '''
        Sets the amplitude of the current
        :param amplitude: float
        '''
        self.__generator.amplitude = amplitude

    amplitude = property(__get_amplitude, __set_amplitude)

    def __get_offset(self):
        '''
        Returns the offset of the current
        '''
        return self.__generator.offset

    def __set_offset(self, offset):
        '''
        Sets the offset of the current
        :param offset: float
        '''
        self.__generator.offset = offset

    offset = property(__get_offset, __set_offset)

    def __get_frequency(self):
        '''
        Returns the frequency of the current
        '''
        return self.__generator.frequency

    def __set_frequency(self, frequency):
        '''
        Sets the frequency of the current
        :param frequency: float
        '''
        self.__generator.frequency = frequency

    frequency = property(__get_frequency, __set_frequency)

    def __get_phase(self):
        '''
        Returns the phase of the current
        '''
        return self.__generator.phase

    def __set_phase(self, phase):
        '''
        Sets the phase of the current
        :param phase: float
        '''
        self.__generator.phase = phase

    phase = property(__get_phase, __set_phase)

    def create_device(self, **params):
        '''
        Creates an alternating current source
        :param params: Dictionary of neuron configuration parameters
        '''
        self.__generator = sim.ACSource(amplitude=params.get('amplitude', 1.0),
                                        offset=params.get('offset', 0.0),
                                        frequency=params.get('frequency',
                                                             10.0),
                                        phase=params.get('phase', 0.0),
                                        start=params.get('start', 0.0),
                                        stop=params.get('stop', None))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)
