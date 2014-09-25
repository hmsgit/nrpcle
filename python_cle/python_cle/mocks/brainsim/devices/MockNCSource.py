'''
Implementation of MockNCSource
moduleauthor: Michael.Weber@fzi.de
'''

from python_cle.brainsim.BrainInterface import INCSource

__author__ = 'MichaelWeber'


class MockNCSource(INCSource):
    """
    Represents ai Mock of a noisy current generator
    :param kwargs: Optional configuration parameters
    """

    #pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a noisy current generator.
        :param params: Dictionary of neuron configuration parameters
        :param mean: Mean value of the noisy current, default: 0.0 nA
        :param stdev: Standard deviation of the noisy current, default: 1.0 nA
        :param dt: Interval between updates of the current amplitude in ms,
            default: simulation time step
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        :param rng: an RNG object from the `pyNN.random` module,
            default: NativeRNG of the back-end
        """
        self.__generator = None
        self.__mean = 0.0
        self.__stdev = 1.0

        self.create_device(**params)

    @property
    def mean(self):
        '''
        Returns the mean of the current
        '''
        return self.__mean

    @mean.setter
    def mean(self, mean):
        '''
        Sets the mean of the current
        :param mean: float
        '''
        self.__mean = mean

    @property
    def stdev(self):
        '''
        Returns the stdev of the current
        '''
        return self.__stdev

    @stdev.setter
    def stdev(self, stdev):
        '''
        Sets the stdev of the current
        :param stdev: float
        '''
        self.__stdev = stdev

    def create_device(self, **params):
        '''
        Create a noisy current source
        :param params: Dictionary of neuron configuration parameters
        :param mean: Mean value of the noisy current, default: 0.0 nA
        :param stdev: Standard deviation of the noisy current, default: 1.0 nA
        :param dt: Interval between updates of the current amplitude in ms,
            default: simulation time step
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        :param rng: an RNG object from the `pyNN.random` module,
            default: NativeRNG of the back-end
        '''

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
