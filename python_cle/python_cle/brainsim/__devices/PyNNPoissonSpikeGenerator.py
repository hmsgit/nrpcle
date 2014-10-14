'''
Implementation of PyNNPoissonSpikeGenerator
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import IPoissonSpikeGenerator
import pyNN.nest as sim
import warnings

__author__ = 'DimitriProbst'


class PyNNPoissonSpikeGenerator(IPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Poisson spike generator.
        :param params: Optional configuration parameters
        :param duration: Duration of spike train, default: infinity
        :param start: Start time of spike train, default: 0.0 ms
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
        self.create_device(**params)

    @property
    def rate(self):
        '''
        Returns the frequency of the Poisson spike generator
        '''
        return self.__generator.get('rate')[0]

    @rate.setter
    def rate(self, value):
        '''
        Sets the frequency of the Poisson spike generator
        :param value: float
        '''
        self.__generator.set('rate', value)

    def create_device(self, **params):
        '''
        Create Poisson spike generator device
        :param params: generator configuration parameters
        :param duration: Duration of spike train, default: infinity
        :param start: Start time of spike train, default: 0.0 ms
        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        '''
        params = {'duration': params.get('duration', float('inf')),
                  'start': params.get('start', 0.0),
                  'rate': params.get('rate', 0.0)}
        self.__generator = sim.Population(1, sim.SpikeSourcePoisson, params)

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
#
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
            delays = sim.RandomDistribution('uniform', [0.1, 0.3])
            connector = sim.AllToAllConnector(weights=weights,
                                              delays=delays)
        proj = sim.Projection(presynaptic_population=self.__generator,
                              postsynaptic_population=neurons,
                              method=connector, source=source,
                              target=target,
                              synapse_dynamics=synapse_dynamics,
                              label=label, rng=rng)
        return proj
