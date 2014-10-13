'''
Implementation of MockLeakyIntegratorAlpha
moduleauthor: Michael.Weber@fzi.de
'''

from python_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha
from . import MockProjection as mp
import warnings

__author__ = 'MichaelWeber'


class MockLeakyIntegratorAlpha(ILeakyIntegratorAlpha):
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
        self.__voltage = params.get('v_rest', 0.0)
        self.__update = [0.0, params.get('v_rest', 0.0)]

        self.create_device(**params)
        self.start_record_voltage()

    @property
    def voltage(self):
        '''
        Returns the membrane voltage of the cell
        '''
        return self.__voltage

    def create_device(self, **params):
        '''
        Creates a LIF neuron with alpha-shaped post synaptic currents
        and current-based synapses
        :param params: Dictionary of neuron configuration parameters
        :param v_thresh: Threshold voltage , default: infinity
        :param cm: Membrane capacitance, default: 1.0 nF
        :param tau_m: Membrane time constant, default: 20.0 ms
        :param tau_syn_E: Excitatory synaptic time constant, default: 0.5 ms
        :param tau_syn_I: Inhibitory synaptic time constant, default: 0.5 ms
        :param v_rest: Resting potential, default: -65.0 mV
        :param v_reset: Reset potential, default: -65.0 mV
        :param tau_refrac: Refractory time constant, default: 0.1 ms
        :param i_offset: Offset current, default: 0.0 nA
        '''

    def start_record_voltage(self):
        '''
        Records the voltage of the neuron
        '''

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

        if type(neurons) == list:
            target = ['excitatory', 'inhibitory']
            if connector is None:
                warnings.warn("Mocked weights and delays are used.",
                              UserWarning)
                connector = []
            proj_exc = mp.MockProjection(presynaptic_population=neurons[0],
                                         postsynaptic_population=self.__cell,
                                         method=connector, source=source,
                                         target=target[0],
                                         synapse_dynamics=synapse_dynamics,
                                         label=label, rng=rng)
            proj_inh = mp.MockProjection(presynaptic_population=neurons[1],
                                         postsynaptic_population=self.__cell,
                                         method=connector, source=source,
                                         target=target[1],
                                         synapse_dynamics=synapse_dynamics,
                                         label=label, rng=rng)
            return [proj_exc, proj_inh]
        else:
            if connector is None:
                warnings.warn("Mocked weights and delays are used.",
                              UserWarning)
                connector = []
            proj = mp.MockProjection(presynaptic_population=neurons,
                                     postsynaptic_population=self.__cell,
                                     method=connector, source=source,
                                     target=target,
                                     synapse_dynamics=synapse_dynamics,
                                     label=label, rng=rng)
            return proj

    def refresh(self, time):
        '''
        Refreshes the voltage value
        :param time: The current simulation time
        '''
        if self.__update[0] is not time:
            self.__update[0] = time
            self.__update[1] = self.voltage
        return self.__update[1]
