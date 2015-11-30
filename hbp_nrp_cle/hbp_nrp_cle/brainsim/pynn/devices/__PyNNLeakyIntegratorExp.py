'''
Implementation of PyNNLeakyIntegratorExp
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorExp
import warnings
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = 'DimitriProbst'


class PyNNLeakyIntegratorExp(AbstractBrainDevice, ILeakyIntegratorExp):
    """
    Represents the membrane potential of a current-based LIF neuron
    with decaying-exponential post-synaptic currents
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to infinity
        by default in order to forbid the neuron to elicit
        spikes.

        :param v_thresh: Threshold voltage , default: infinity
        :param cm: Membrane capacitance, default: 1.0 nF
        :param tau_m: Membrane time constant, default: 20.0 ms
        :param tau_syn_E: Excitatory synaptic time constant, default: 0.5 ms
        :param tau_syn_I: Inhibitory synaptic time constant, default: 0.5 ms
        :param v_rest: Resting potential, default: 0.0 mV
        :param v_reset: Reset potential, default: 0.0 mV
        :param tau_refrac: Refractory time constant, default: 0.1 ms
        :param i_offset: Offset current, default: 0.0 nA
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
        self._cell = None
        self._voltage = None

        self.create_device(params)
        self.start_record_voltage()

    @property
    def voltage(self):
        """
        Returns the membrane voltage of the cell
        """
        return self._voltage

    def create_device(self, params):
        """
        Creates a LIF neuron with decaying-exponential post-synaptic currents
        and current-based synapses

        :param params: Dictionary of neuron configuration parameters
        :param v_thresh: Threshold voltage , default: infinity
        :param cm: Membrane capacitance, default: 1.0 nF
        :param tau_m: Membrane time constant, default: 20.0 ms
        :param tau_syn_E: Excitatory synaptic time constant, default: 0.5 ms
        :param tau_syn_I: Inhibitory synaptic time constant, default: 0.5 ms
        :param v_rest: Resting potential, default: 0.0 mV
        :param v_reset: Reset potential, default: 0.0 mV
        :param tau_refrac: Refractory time constant, default: 0.1 ms
        :param i_offset: Offset current, default: 0.0 nA
        """
        cellparams = {'v_thresh': params.get('v_thresh', float('inf')),
                      'cm': params.get('cm', 1.0),
                      'tau_m': params.get('tau_m', 20.0),
                      'tau_syn_E': params.get('tau_syn_E', 0.5),
                      'tau_syn_I': params.get('tau_syn_I', 0.5),
                      'v_rest': params.get('v_rest', 0.0),
                      'v_reset': params.get('v_reset', 0.0),
                      'tau_refrac': params.get('tau_refrac', 0.1),
                      'i_offset': params.get('i_offset', 0.0)}
        self._cell = sim.Population(1, sim.IF_curr_exp, cellparams)
        sim.initialize(self._cell, 'v', self._cell[0].v_rest)

    def start_record_voltage(self):
        """
        Records the voltage of the neuron
        """
        self._cell.record_v()

    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        :param params: Optional configuration parameters
        :param connector: a PyNN Connector object
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

        if connector is None:
            warnings.warn("Default weights and delays are used.",
                          UserWarning)
            if target == 'excitatory':
                weights = sim.RandomDistribution('uniform', [0.0, 0.01])
            else:
                weights = sim.RandomDistribution('uniform', [-0.01, -0.0])
            delays = sim.RandomDistribution('uniform', [0.1, 2.0])
            connector = sim.AllToAllConnector(weights=weights,
                                              delays=delays)
        proj = sim.Projection(presynaptic_population=neurons,
                              postsynaptic_population=self._cell,
                              method=connector, source=source,
                              target=target,
                              synapse_dynamics=synapse_dynamics,
                              label=label, rng=rng)
        return proj

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """

        self._voltage = self._cell.get_v()[-1, -1]
