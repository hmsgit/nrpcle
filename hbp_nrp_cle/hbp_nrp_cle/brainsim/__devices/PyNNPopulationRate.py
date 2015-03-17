'''
Implementation of PyNNPopulationRate
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import IPopulationRate
import pyNN.nest as sim
# pylint: disable=E0611
from scipy.integrate import simps
import numpy as np

__author__ = 'DimitriProbst'


class PyNNPopulationRate(IPopulationRate):
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
        self.__rate = None

        self.create_device(**params)
        self.calculate_weight()
        self.start_record_rate()

    @property
    def rate(self):
        """
        Returns the population firing rate
        """
        return self.__rate

    def create_device(self, **params):
        """
        Creates a LIF neuron with decaying-exponential post-synaptic currents
        and current-based synapses.

        :param params: Dictionary of neuron configuration parameters
        :param tau_rise: Rising time constant, default: 10.0 ms
        :param tau_fall: Falling time constant, default: 20.0 ms
        """
        cellparams = {'v_thresh': float('inf'),
                      'cm': 1.0,
                      'tau_m': params.get('tau_fall', 20.0),
                      'tau_syn_E': params.get('tau_rise', 10.0),
                      'v_rest': 0.0}
        self.__cell = sim.Population(1, sim.IF_curr_exp, cellparams)
        sim.initialize(self.__cell, 'v', self.__cell[0].v_rest)

    def calculate_weight(self):
        """
        Calculates the weight of a neuron from the population to the device
        such that the area below the resulting PSP is 1. The exact shape of a
        PSP can be found e.g. in Bytschok, I., Diploma thesis.
        """
        tau_c = (1. / self.__cell[0].tau_syn_E - 1. / self.__cell[0].tau_m) ** -1
        t_end = -np.log(1e-10) * self.__cell[0].tau_m
        x_new = np.arange(0., t_end, 0.1)
        y_new = tau_c / self.__cell[0].cm * (np.exp(
            -x_new / self.__cell[0].tau_m) - np.exp(
            -x_new / self.__cell[0].tau_syn_E))
        self.__weight = 1.0 / simps(y_new, dx=sim.state.dt)

    def start_record_rate(self):
        """
        Records the rate of a neuronal population
        """
        self.__cell.record_v()

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        connector = sim.AllToAllConnector(weights=self.__weight * 1000,
                                          delays=sim.state.dt)
        sim.Projection(presynaptic_population=neurons,
                       postsynaptic_population=self.__cell,
                       method=connector, target='excitatory')

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the rate value

        :param time: The current simulation time
        """
        ### HACK ###
        # The usual PyNN get_v() call
        # return self.__cell.get_v()[-1, -1]
        # takes too much time.
        # In the meantime, until the PyNN call gets fixed, we can use the
        # function of the NEST back-end.
        self.__rate = sim.simulator.nest.GetStatus([self.__cell[0]])[0]['V_m']
