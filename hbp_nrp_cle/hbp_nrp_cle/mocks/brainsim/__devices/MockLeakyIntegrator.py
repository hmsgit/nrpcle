'''
This module contains a mock implementation for the two leaky integrator devices
ILeakyIntegratorAlpha and ILeakyIntegratorExp
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, ILeakyIntegratorExp
import warnings

__author__ = 'MichaelWeber'


class MockLeakyIntegrator(AbstractMockBrainDevice):
    """
    Mock device for leaky integrators
    """

    default_parameters = {
        "target": "excitatory",
        "v_rest": 0.0,
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
        super(MockLeakyIntegrator, self).__init__(**params)

        self.__voltage = self._parameters["v_rest"]
        self.__update = self._parameters["updates"]

    @property
    def voltage(self):
        """
        Returns the membrane voltage of the cell
        """
        return self.__voltage

    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        if hasattr(self.__update, '__getitem__'):
            while len(self.__update) > 0 and isinstance(self.__update[0], tuple) \
                    and time >= self.__update[0][0]:
                self.__voltage = self.__update[0][1]
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

        :param updates: A new list of update information. This list must consist of tuples where
         the first argument is the simulation time and the second argument is the voltage
        """
        self.__update = updates
        if updates is None:
            self.__update = []


class MockLeakyIntegratorAlpha(MockLeakyIntegrator, ILeakyIntegratorAlpha):
    """
    Mock device representing a leaky integrator using the membrane potential of a current-based
    LIF neuron with alpha-shaped post synaptic currents
    """


class MockLeakyIntegratorExp(MockLeakyIntegrator, ILeakyIntegratorExp):
    """
    Mock device representing a leaky integrator using the membrane potential of a current-based
    LIF neuron with alpha-shaped post synaptic currents
    """
