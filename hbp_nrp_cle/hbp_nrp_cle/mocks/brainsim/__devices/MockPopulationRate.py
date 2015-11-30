'''
This module contains the mock implementations for a population rate device which can be set up to
provide predefined values at certain points in time. The device can be used as mock input for
transfer functions, dispensing with the need to run a brain simulation to test them.
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IPopulationRate
import warnings

__author__ = 'PatrikScheidecker'


class MockPopulationRate(AbstractMockBrainDevice, IPopulationRate):
    """
    Represents the rate of a population of LIF neurons by
    measuring and normalizing the membrane potential of a
    leaky integrator with decaying-exponential post-synaptic currents
    """

    default_parameters = {
        "rate": 0.0,
        "updates": []
    }

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to "infinity"
        is set to infinity by default in order to forbid the neuron to elicit
        spikes.

        :param params: Dictionary of neuron configuration parameters
        """
        super(MockPopulationRate, self).__init__(**params)

        self.__rate = self._parameters["rate"]
        self.__update = self._parameters["updates"]

    @property
    def rate(self):
        """
        Returns the population firing rate
        """
        return self.__rate

    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        if hasattr(self.__update, '__getitem__'):
            while len(self.__update) > 0 and isinstance(self.__update[0], tuple)\
                    and time >= self.__update[0][0]:
                self.__rate = self.__update[0][1]
                self.__update = self.__update[1:]
        else:
            warnings.warn("Updates schedules must be sorted lists of tuples")

    @property
    def updates(self):
        """
        Gets the scheduled updates for this device

        :return: A list of tuples when the mock device should be updated
        """
        return self.__update

    @updates.setter
    def updates(self, updates):
        """
        Sets the scheduled updates for this device

        :param updates:A new list of update information. This list must consist of tuples where
         the first argument is the simulation time and the second argument is the population rate
        """
        self.__update = updates
        if updates is None:
            self.__update = []
