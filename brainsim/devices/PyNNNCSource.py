'''
Implementation of PyNNNCSource
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import INCSource
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNNCSource(INCSource):
    """
    Represents a noisy current generator
    :param kwargs: Optional configuration parameters
    """

    def __init__(self, **params):
        """
        Initializes a noisy current generator.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__generator = None

        self.create_device(**params)

    def __get_mean(self):
        '''
        Returns the mean of the current
        '''
        return self.__generator.mean

    def __set_mean(self, mean):
        '''
        Sets the mean of the current
        :param mean: float
        '''
        self.__generator.mean = mean

    mean = property(__get_mean, __set_mean)

    def __get_stdev(self):
        '''
        Returns the stdev of the current
        '''
        return self.__generator.stdev

    def __set_stdev(self, stdev):
        '''
        Sets the stdev of the current
        :param stdev: float
        '''
        self.__generator.stdev = stdev

    stdev = property(__get_stdev, __set_stdev)

    def create_device(self, **params):
        '''
        Create a noisy current source
        :param params: Dictionary of neuron configuration parameters
        '''
        self.__generator = sim.NoisyCurrentSource(
            mean=params.get('mean', 0.0),
            stdev=params.get('stdev', 1.0),
            dt=params.get('dt', None),
            start=params.get('start', 0.0),
            stop=params.get('stop', None),
            rng=params.get('rng', sim.NativeRNG()))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)
