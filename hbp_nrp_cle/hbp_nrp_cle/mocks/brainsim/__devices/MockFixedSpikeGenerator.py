'''
Implementation of MockFixedSpikeGenerator
moduleauthor: scheidecker@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator

__author__ = 'PatrikScheidecker'


class MockFixedSpikeGenerator(IFixedSpikeGenerator):
    """
    Represents a Fixed Spike Generator, which generates spikes with a
    fixed predefined interval duration.
    """
    #pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.
        :param rate: Rate of spike generation
        """
        self.__rate = params.get('rate', 1.0)
        self.__history = []

    @property
    def rate(self):
        """
        Returns the spike generation rate
        """
        return self.__rate

    @rate.setter
    def rate(self, rate):
        """
        Sets the spike generation rate
        :param rate: float
        """
        self.__rate = rate
        self.__history.append(rate)

    @property
    def history(self):
        """
        Lists the rates assigned to this device
        :return: A list of float values
        """
        return self.__history
