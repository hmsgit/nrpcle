'''
Implementation of PyNNFixedSpikeGenerator
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator
import pyNN.nest as sim
import nest
import numpy as np
import warnings

__author__ = 'DimitriProbst'


class PyNNFixedSpikeGenerator(AbstractBrainDevice, IFixedSpikeGenerator):
    """
    Represents a spike generator which generated equidistant
    spike times at a given frequency
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Fixed spike generator.

        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        :param connector: a PyNN Connector object, or, if neurons is
            a list of two populations, a list of two Connector objects
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_dynamics: a PyNN SynapseDy
        :param label: label of the Projection object
        :param rng: RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        self.__generator = None
        self.__currentsource = None
        self.__rate = params.get('rate', 0.0)
        self.create_device()

    @property
    def rate(self):
        """
        Returns the frequency of the Fixed spike generator
        """
        return self.__rate

    @rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Fixed spike generator

        :param value: float
        """
        amplitude = self.set_current(value)
        self.__currentsource.amplitude = amplitude
        # PyNN<0.8 does not support changing current source reconfiguration
        # pylint: disable=W0212
        nest.SetStatus(self.__currentsource._device, {'amplitude': 1000.0 * amplitude})

    def create_device(self):
        """
        Create a fixed spike-distance device
        """
        cellparams = {'cm': 1.0,
                      'tau_m': 1000.0,
                      'tau_refrac': sim.state.dt,
                      'v_thresh': -50.0,
                      'v_reset': -100.0,
                      'v_rest': -100.0}
        self.__generator = sim.Population(1, sim.IF_curr_exp, cellparams)
        sim.initialize(self.__generator, 'v', self.__generator[0].v_rest)
        self.__currentsource = sim.DCSource(
            amplitude=self.set_current(self.__rate))
        self.__currentsource.inject_into(self.__generator)

    def set_current(self, rate):
        """
        Returns current in nA corresponding to frequency "rate"

        :param rate: Frequency in Hz
        """
        tau_m = 1000.0
        tau_refrac = sim.state.dt
        cm = 1.0
        v_thresh = -50.0
        v_rest = -100.0
        nom = (cm / tau_m) * (v_thresh - v_rest)
        denom = 1.0
        if rate != 0.0:
            inter_time = 1000.0 / rate
            if inter_time < 10 * tau_refrac:
                rate = 100.0 / tau_refrac
                inter_time = 10 * tau_refrac
            denom = 1.0 - np.exp((tau_refrac - inter_time) / tau_m)
        self.__rate = rate
        return nom / denom

    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        :param params: optional configuration parameters
        :param connector: a PyNN Connector object
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_dynamics: a PyNN SynapseDy
        :param label: label of the Projection object
        :param rng: RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        connector = params.get('connector', None)
        source = params.get('source', None)
        target = params.get('target', 'excitatory')
        synapse_dynamics = params.get('synapse_dynamics', None)
        label = params.get('label', None)
        rng = params.get('rng', None)

#        if type(neurons) == list:
#            target = ['excitatory', 'inhibitory']
#            if connector is None:
#                warnings.warn("Default weights and delays are used.",
#                              UserWarning)
#                connector = []
#                weights = sim.RandomDistribution('uniform', [0.0, 0.01])
#                delays = sim.RandomDistribution('uniform', [0.1, 2.0])
#                connector.append(sim.AllToAllConnector(weights=weights,
#                                                       delays=delays))
#                if neurons[1].conductance_based:
#                    weights = sim.RandomDistribution('uniform', [0.0,
#                                                                 0.01])
#                else:
#                    weights = sim.RandomDistribution('uniform', [-0.01,
#                                                                 -0.0])
#                connector.append(sim.AllToAllConnector(weights=weights,
#                                                       delays=delays))
#            proj_exc = sim.Projection(presynaptic_population=self.__generator,
#                                      postsynaptic_population=neurons[0],
#                                      method=connector[0], source=source,
#                                      target=target[0],
#                                      synapse_dynamics=synapse_dynamics,
#                                      label=label, rng=rng)
#            proj_inh = sim.Projection(presynaptic_population=self.__generator,
#                                      postsynaptic_population=neurons[1],
#                                      method=connector[1], source=source,
#                                      target=target[1],
#                                      synapse_dynamics=synapse_dynamics,
#                                      label=label, rng=rng)
#            return [proj_exc, proj_inh]
#        else:
        if connector is None:
            warnings.warn("Default weights and delays are used.",
                          UserWarning)
            if target == 'excitatory':
                weights = sim.RandomDistribution('uniform', [0.0, 0.01])
            else:
                if neurons.conductance_based:
                    weights = sim.RandomDistribution('uniform', [0.0, 0.01])
                else:
                    weights = sim.RandomDistribution('uniform', [-0.01, -0.0])
            delays = sim.RandomDistribution('uniform', [0.1, 2.0])
            connector = sim.AllToAllConnector(weights=weights,
                                              delays=delays)
        proj = sim.Projection(presynaptic_population=self.__generator,
                              postsynaptic_population=neurons,
                              method=connector, source=source,
                              target=target,
                              synapse_dynamics=synapse_dynamics,
                              label=label, rng=rng)
        return proj
