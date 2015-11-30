'''
This module contains the PyNN-specific implementation of the ILeakyIntegratorAlpha and the
ILeakyIntegratorExp device.
'''

from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, ILeakyIntegratorExp
from .__PyNNLeakyIntegrator import PyNNLeakyIntegrator
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = "Sebastian Krach"


class PyNNLeakyIntegratorAlpha(PyNNLeakyIntegrator, ILeakyIntegratorAlpha):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    default_parameters = {
        'v_thresh': float('inf'),
        'cm': 1.0,
        'tau_m': 10.0,
        'tau_syn_E': 2.,
        'tau_syn_I': 2.,
        'v_rest': 0.0,
        'v_reset': 0.0,
        'tau_refrac': 0.1,
        'i_offset': 0.0,
        'connector': None,
        'weights': None,
        'delays': 0.1,
        'source': None,
        'target': 'excitatory',
        'synapse_dynamics': None,
        'label': None,
        'rng': None
    }

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to infinity
        by default in order to forbid the neuron to elicit spikes.

        :param v_thresh: Threshold voltage , default: infinity
        :param cm: Membrane capacitance, default: 1.0 nF
        :param tau_m: Membrane time constant, default: 20.0 ms
        :param tau_syn_E: Excitatory synaptic time constant, default: 0.5 ms
        :param tau_syn_I: Inhibitory synaptic time constant, default: 0.5 ms
        :param v_rest: Resting potential, default: 0.0 mV
        :param v_reset: Reset potential, default: 0.0 mV
        :param tau_refrac: Refractory time constant, default: 0.1 ms
        :param i_offset: Offset current, default: 0.0 nA
        :param connector: a PyNN Connector object which is used to connect the neuron population
                of the brain model to the integrator neuron of this device.
                Default: AllToAllConnector
        :param weights: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified weights. Default: 0.01 (excitatory) or -0.01 (else)
        :param delays: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified delay. Default: 0.1
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: "excitatory" or "inhibitory". Default: "excitatory"
        :param synapse_dynamics: (optional) a PyNN SynapseDynamics object
        :param label: (optional) label of the Projection object
        :param rng: (optional) RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNLeakyIntegratorAlpha, self).__init__(**params)

    @staticmethod
    def _get_cell_type():
        """
        Returns the cell type of the neuron to be created by this device. In this case the neuron
        to be created adheres to PyNN's "Leaky integrate and fire model with fixed threshold and
        alpha-function-shaped post-synaptic current"

        :return: PyNN IF_curr_alpha cell type
        """
        return sim.IF_curr_alpha

    def _get_connector_weight(self):
        """
        Returns the default connector weight in case no explicit weight is specified as parameter

        :return: the weight of the synaptic connection
        """
        if self._parameters["target"] == 'excitatory':
            return 0.01
        else:
            return -0.01


class PyNNLeakyIntegratorExp(PyNNLeakyIntegrator, ILeakyIntegratorExp):
    """
    Represents the membrane potential of a current-based LIF neuron
    with decaying-exponential post-synaptic currents.
    """

    default_parameters = {'v_thresh': float('inf'),
                          'cm': 1.0,
                          'tau_m': 20.0,
                          'tau_syn_E': .5,
                          'tau_syn_I': .5,
                          'v_rest': 0.0,
                          'v_reset': 0.0,
                          'tau_refrac': 0.1,
                          'i_offset': 0.0,
                          'connector': None,
                          'weights': None,
                          'delays': sim.RandomDistribution('uniform', [0.1, 2.0]),
                          'source': None,
                          'target': 'excitatory',
                          'synapse_dynamics': None,
                          'label': None,
                          'rng': None}

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to infinity
        by default in order to forbid the neuron to elicit spikes.

        :param v_thresh: Threshold voltage, default: infinity
        :param cm: Membrane capacitance, default: 1.0 nF
        :param tau_m: Membrane time constant, default: 20.0 ms
        :param tau_syn_E: Excitatory synaptic time constant, default: 2.0 ms
        :param tau_syn_I: Inhibitory synaptic time constant, default: 2.0 ms
        :param v_rest: Resting potential, default: 0.0 mV
        :param v_reset: Reset potential, default: 0.0 mV
        :param tau_refrac: Refractory time constant, default: 0.1 ms
        :param i_offset: Offset current, default: 0.0 nA
        :param connector: a PyNN Connector object which is used to connect the neuron population
                of the brain model to the integrator neuron of this device.
                Default: AllToAllConnector
        :param weights: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified weights.
                Default: randomly distributed in [0, 0.01] (excitatory) or [-0.01, 0] (else)
        :param delays: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified delay.
                Default: randomly distributed in [0.1, 2.0]
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: "excitatory" or "inhibitory". Default: "excitatory"
        :param synapse_dynamics: (optional) a PyNN SynapseDynamics object
        :param label: (optional) label of the Projection object
        :param rng: (optional) RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNLeakyIntegratorExp, self).__init__(**params)

    @staticmethod
    def _get_cell_type():
        """
        Returns the cell type of the neuron to be created by this device. In this case the neuron
        to be created adheres to PyNN's "Leaky integrate and fire model with fixed threshold and
        decaying-exponential post-synaptic current"

        :return: PyNN IF_curr_exp cell type
        """
        return sim.IF_curr_exp

    def _get_connector_weight(self):
        """
        Returns the default connector weight in case no explicit weight is specified as parameter

        :return: the weight of the synaptic connection
        """
        if self._parameters["target"] == 'excitatory':
            return sim.RandomDistribution('uniform', [0., 0.01])
        else:
            return sim.RandomDistribution('uniform', [-0.01, -0.])
