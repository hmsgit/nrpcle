'''
Implementation of PyNNDCSource
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import IDCSource
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNDCSource(IDCSource):
    """
    Represents a direct current generator
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a direct current generator.

        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """
        self.__generator = None
        self.create_device(**params)

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self.__generator.amplitude

    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        self.__generator.amplitude = value

    def create_device(self, **params):
        """
        Create a direct current source

        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """
        self.__generator = sim.DCSource(amplitude=params.get('amplitude', 1.0),
                                        start=params.get('start', 0.0),
                                        stop=params.get('stop', None))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)