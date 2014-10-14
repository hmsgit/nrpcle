'''
Implementation of MockFixedSpikeGenerator
moduleauthor: scheidecker@fzi.de
'''

from python_cle.brainsim.BrainInterface import IFixedSpikeGenerator
from . import MockProjection as mp

__author__ = 'PatrikScheidecker'


class MockFixedSpikeGenerator(IFixedSpikeGenerator):
    """
    Represents a Fixed Spike Generator, which generates spikes with a
    fixed predefined interval duration.
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Fixed spike generator.
        :param params: Optional configuration parameters
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
        '''
        Returns the frequency of the Fixed spike generator
        '''
        return self.__rate

    @rate.setter
    def rate(self, value):
        '''
        Sets the frequency of the Fixed spike generator
        :param value: float
        '''
        self.__rate = value

    def create_device(self):
        '''
        Create a fixed spike-distance device
        '''

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
        connector = params.get('connector', None)
        source = params.get('source', None)
        target = params.get('target', 'excitatory')
        synapse_dynamics = params.get('synapse_dynamics', None)
        label = params.get('label', None)
        rng = params.get('rng', None)

        if type(neurons) == list:
            target = ['excitatory', 'inhibitory']
            proj_exc = mp.MockProjection(presynaptic_population=self.__generator,
                                         postsynaptic_population=neurons[0],
                                         method=None, source=source,
                                         target=target[0],
                                         synapse_dynamics=synapse_dynamics,
                                         label=label, rng=rng)
            proj_inh = mp.MockProjection(presynaptic_population=self.__generator,
                                         postsynaptic_population=neurons[1],
                                         method=None, source=source,
                                         target=target[1],
                                         synapse_dynamics=synapse_dynamics,
                                         label=label, rng=rng)
            return [proj_exc, proj_inh]
        else:
            proj = mp.MockProjection(presynaptic_population=self.__generator,
                                     postsynaptic_population=neurons,
                                     method=connector, source=source,
                                     target=target,
                                     synapse_dynamics=synapse_dynamics,
                                     label=label, rng=rng)
            return proj
