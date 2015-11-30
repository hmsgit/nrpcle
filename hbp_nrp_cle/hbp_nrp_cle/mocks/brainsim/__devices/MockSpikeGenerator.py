'''
This module contains mock implementations for a spike generator device which record the changes
to the device parameters and can be used to track transfer function activity without the need to
run an entire brain simulation.
'''


from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, IPoissonSpikeGenerator

__author__ = 'PatrikScheidecker'


class MockSpikeGenerator(AbstractMockBrainDevice, IFixedSpikeGenerator):
    """
    Mock device base class for spike generators.
    """

    default_parameters = {
        "target": "excitatory",
        "rate": 1.0
    }

    # pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param rate: Rate of spike generation
        """
        super(MockSpikeGenerator, self).__init__(**params)

        self.__rate = self._parameters["rate"]
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


class MockFixedSpikeGenerator(MockSpikeGenerator, IFixedSpikeGenerator):
    """
    Mock device representing a Fixed Spike Generator, which generates spikes with a
    fixed predefined interval duration.
    """


class MockPoissonSpikeGenerator(MockSpikeGenerator, IPoissonSpikeGenerator):
    """
    Mock device representing a Poisson Spike Generator, which generates spikes with according
    to a poisson distribution.
    """
