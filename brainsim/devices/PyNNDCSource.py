'''
Implementation of PyNNDCSource
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IDCSource
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNDCSource(IDCSource):
    """
    Represents a direct current generator
    :param kwargs: Optional configuration parameters
    """

    def __init__(self, params):
        """
        Initializes a direct current generator.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__generator = None
        self.create_device(params)

    def __get_amplitude(self):
        return self.__generator.amplitude

    def __set_amplitude(self, amplitude):
        self.__generator.amplitude = amplitude

    amplitude = property(__get_amplitude, __set_amplitude)

    def create_device(self, params):
        '''
        Create a direct current source
        :param params: Dictionary of neuron configuration parameters
        '''
        self.__generator = sim.DCSource(amplitude=params.get('amplitude', 1.0),
                                        start=params.get('start', 0.0),
                                        stop=params.get('stop', None))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)
