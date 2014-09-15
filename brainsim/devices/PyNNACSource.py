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

    def __init__(self, params):
        """
        Initializes an alternating current generator.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__generator = None

        self.create_device(params)

    def __get_amplitude(self):
        return self.__generator.amplitude

    def __set_amplitude(self, amplitude):
        self.__generator.amplitude = amplitude

    amplitude = property(__get_amplitude, __set_amplitude)

    def __get_offset(self):
        return self.__generator.offset

    def __set_offset(self, offset):
        self.__generator.offset = offset

    offset = property(__get_offset, __set_offset)

    def __get_frequency(self):
        return self.__generator.frequency

    def __set_frequency(self, frequency):
        self.__generator.frequency = frequency

    frequency = property(__get_frequency, __set_frequency)

    def __get_phase(self):
        return self.__generator.phase

    def __set_phase(self, phase):
        self.__generator.phase = phase

    phase = property(__get_phase, __set_phase)

    def create_device(self, params):
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
