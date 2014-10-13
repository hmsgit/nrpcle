'''
Implementation of MockPopulationRate
moduleauthor: scheidecker@fzi.de
'''

from python_cle.brainsim.BrainInterface import IPopulationRate
from . import MockProjection as mp

__author__ = 'PatrikScheidecker'


class MockPopulationRate(IPopulationRate):
    """
    Represents the rate of a population of LIF neurons by
    measuring and normalizing the membrane potential of a
    leaky integrator with decaying-exponential post-synaptic currents
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is read out and
        normalized. The obligatory threshold voltage 'v_thresh' is set to
        infinity. The rising time constant is always smaller
        than the falling time constant. If tau_rise > tau_fall,
        "tau_rise" is interpreted as falling time constant and
        "tau_fall" as rising time constant.
        :param params: Dictionary of neuron configuration parameters
        :param tau_rise: Rising time constant, default: 10.0 ms
        :param tau_fall: Falling time constant, default: 20.0 ms
        """
        self.__cell = None
        self.__weight = None
        self.__voltage = 0.0
        self.__update = [0.0, 0.0]

        self.create_device(**params)
        self.calculate_weight()
        self.start_record_rate()
        self.__rate = 0.0

    @property
    def rate(self):
        '''
        Returns the population firing rate
        '''
        return self.__rate

    def create_device(self, **params):
        '''
        Creates a LIF neuron with decaying-exponential post-synaptic currents
        and current-based synapses.
        :param params: Dictionary of neuron configuration parameters
        :param tau_rise: Rising time constant, default: 10.0 ms
        :param tau_fall: Falling time constant, default: 20.0 ms
        '''


    def calculate_weight(self):
        '''
        Calculates the weight of a neuron from the population to the device
        such that the area below the resulting PSP is 1. The exact shape of a
        PSP can be found e.g. in Bytschok, I., Diploma thesis.
        '''

    def start_record_rate(self):
        '''
        Records the rate of a neuronal population
        '''

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        device.
        :param neurons: must be a Population, PopulationView or
            Assembly object
        """

        mp.MockProjection(presynaptic_population=neurons,
                          postsynaptic_population=self.__cell,
                          method=None, target='excitatory')

    def refresh(self, time):
        '''
        Refreshes the voltage value
        :param time: The current simulation time
        '''
        if self.__update[0] is not time:
            self.__update[0] = time
            self.__update[1] = self.rate
        return self.__update[1]
