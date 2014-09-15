'''
Implementation of PyNNPoissonSpikeGenerator
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IPoissonSpikeGenerator
import pyNN.nest as sim
import warnings

__author__ = 'DimitriProbst'


class PyNNPoissonSpikeGenerator(IPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    def __init__(self, params):
        """
        Initializes a Poisson spike generator.
        :param params: Optional configuration parameters
        """
        self.__generator = None
        self.create_device(params)

    def __get_rate(self):
        return self.__generator.get('rate')[0]

    def __set_rate(self, rate):
        self.__generator.set('rate', rate)

    rate = property(__get_rate, __set_rate)

    def create_device(self, params):
        '''
        Create Poisson spike generator device
        :param params: generator configuration parameters
        '''
        params = {'duration': params.get('duration', float('inf')),
                  'start': params.get('start', 0.0),
                  'rate': params.get('rate', 0.0)}
        self.__generator = sim.Population(1, sim.SpikeSourcePoisson, params)

    def connect(self, neurons, **kwargs):
        """
        Connects the neurons specified in the list "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        connector = kwargs.get('connector', None)
        source = kwargs.get('source', None)
        target = kwargs.get('target', 'excitatory')
        synapse_dynamics = kwargs.get('synapse_dynamics', None)
        label = kwargs.get('label', None)
        rng = kwargs.get('rng', None)

        if connector is None:
            warnings.warn("Default weights and delays are used.", UserWarning)
            weights = sim.RandomDistribution('uniform', [0.0, 0.01])
            delays = sim.RandomDistribution('uniform', [0.1, 2.0])
            connector = sim.AllToAllConnector(weights=weights, delays=delays)
        return sim.Projection(presynaptic_population=self.__generator,
                              postsynaptic_population=neurons,
                              method=connector, source=source, target=target,
                              synapse_dynamics=synapse_dynamics, label=label,
                              rng=rng)
