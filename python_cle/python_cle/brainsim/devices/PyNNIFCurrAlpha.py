'''
Implementation of PyNNIFCurrAlpha
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import IIFCurrAlpha
import warnings
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNIFCurrAlpha(IIFCurrAlpha):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to "infinity"
        is set to infinity by default in order to forbid the neuron to elicit
        spikes.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__cell = None
        self.__voltage = params.get('v_rest', -65.0)
        self.__update = [0.0, params.get('v_rest', -65.0)]

        self.create_device(**params)
        self.start_record_voltage()

    def __get_voltage(self):
        '''
        Returns the membrane voltage of the cell
        '''
        return self.__cell.get_v()[-1, -1]

    voltage = property(__get_voltage)

    def create_device(self, **params):
        '''
        Creates a LIF neuron with alpha-shaped post synaptic currents
        and current-based synapses
        :param params: Dictionary of neuron configuration parameters
        '''
        cellparams = {'v_thresh': params.get('v_thresh', float('inf')),
                      'cm': params.get('cm', 1.0),
                      'tau_m': params.get('tau_m', 20.0),
                      'tau_syn_E': params.get('tau_syn_E', 0.5),
                      'tau_syn_I': params.get('tau_syn_I', 0.5),
                      'v_rest': params.get('v_rest', -65.0),
                      'v_reset': params.get('v_reset', -65.0),
                      'tau_refrac': params.get('tau_refrac', 0.1),
                      'i_offset': params.get('i_offset', 0.0)}
        self.__cell = sim.Population(1, sim.IF_curr_alpha, cellparams)

    def start_record_voltage(self):
        '''
        Records the voltage of the neuron
        '''
        self.__cell.record_v()

    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.
        :param neurons: must be a Population, PopulationView or
            Assembly object, or a list of two objects [excitatory, inhibitory]
        :param params: Optional configuration parameters
        :param connector: a PyNN Connector object, or, if neurons is
            a list of two populations, a list of two Connector objects
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_dynamics: a PyNN SynapseDynamics object
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
        w_min = 0.0
        w_max = 0.01
        d_min = 0.1
        d_max = 2.0

        if type(neurons) == list:
            target = ['excitatory', 'inhibitory']
            if connector is None:
                warnings.warn("Default weights and delays are used.",
                              UserWarning)
                connector = []
                weights = sim.RandomDistribution('uniform', [w_min, w_max])
                delays = sim.RandomDistribution('uniform', [d_min, d_max])
                connector.append(sim.AllToAllConnector(weights=weights,
                                                       delays=delays))
                weights = sim.RandomDistribution('uniform', [-w_max, -w_min])
                connector.append(sim.AllToAllConnector(weights=weights,
                                                       delays=delays))
            sim.Projection(presynaptic_population=neurons[0],
                           postsynaptic_population=self.__cell,
                           method=connector[0], source=source,
                           target=target[0], synapse_dynamics=synapse_dynamics,
                           label=label, rng=rng)
            sim.Projection(presynaptic_population=neurons[1],
                           postsynaptic_population=self.__cell,
                           method=connector[1], source=source,
                           target=target[1], synapse_dynamics=synapse_dynamics,
                           label=label, rng=rng)
        else:
            if connector is None:
                warnings.warn("Default weights and delays are used.",
                              UserWarning)
                if target == 'excitatory':
                    weights = sim.RandomDistribution('uniform', [w_min, w_max])
                else:
                    weights = sim.RandomDistribution('uniform', [-w_max,
                                                                 -w_min])
                delays = sim.RandomDistribution('uniform', [d_min, d_max])
                connector = sim.AllToAllConnector(weights=weights,
                                                  delays=delays)
            sim.Projection(presynaptic_population=neurons,
                           postsynaptic_population=self.__cell,
                           method=connector, source=source,
                           target=target, synapse_dynamics=synapse_dynamics,
                           label=label, rng=rng)

    def refresh(self, time):
        '''
        Refreshes the voltage value
        :param time: The current simulation time
        '''
        if self.__update[0] is not time:
            self.__update[0] = time
            self.__update[1] = self.__get_voltage()
        return self.__update[1]
