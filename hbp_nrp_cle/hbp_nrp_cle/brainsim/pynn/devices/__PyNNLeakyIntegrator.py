'''
This module contains the shared codebase for all PyNN-specific implementation of leaky integrator
devices.
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = 'Dimitri Probst, Sebastian Krach, Georg Hinkel'


class PyNNLeakyIntegrator(AbstractBrainDevice):
    """
    Abstract super class of brain devices that represent neuronal spike activity by reading out
    the membrane potential of additional integrate and fire neurons. The concrete type of neuron
    depends on the concrete realization of this class.
    """

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to infinity
        by default in order to forbid the neuron to elicit spikes.

        :param v_thresh: Threshold voltage
        :param cm: Membrane capacitance
        :param tau_m: Membrane time constant
        :param tau_syn_E: Excitatory synaptic time constant
        :param tau_syn_I: Inhibitory synaptic time constant
        :param v_rest: Resting potential
        :param v_reset: Reset potential
        :param tau_refrac: Refractory time constant
        :param i_offset: Offset current
        :param connector: a PyNN Connector object, or, if neurons is
            a list of two populations, a list of two Connector objects
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory']
        :param synapse_dynamics: a PyNN SynapseDynamics object
        :param label: label of the Projection object
        :param rng: RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNLeakyIntegrator, self).__init__(**params)

        self._cell = None
        self._voltage = None

        self.create_device()
        self.start_record_voltage()

    @staticmethod
    def _get_cell_type():
        """
        Returns the cell type of the neuron to be created by this device
        :return: a PyNN cell type
        """
        raise NotImplementedError("The cell type has to be specified in a concrete implementation")

    def _get_connector_weight(self):
        """
        Returns the default connector weight in case no explicit weight is specified as parameter
        :return: the weight of the synaptic connection
        """
        raise NotImplementedError("The cell type has to be specified in a concrete implementation")

    @property
    def voltage(self):
        """
        Returns the membrane voltage of the cell
        """
        return self._voltage

    def create_device(self):
        """
        Creates a LIF neuron with alpha-shaped post synaptic currents
        and current-based synapses
        """
        self._cell = sim.Population(1,
                                    self._get_cell_type(),
                                    self.get_parameters('v_thresh',
                                                        'cm',
                                                        'tau_m',
                                                        'tau_syn_E',
                                                        'tau_syn_I',
                                                        'v_rest',
                                                        'v_reset',
                                                        'tau_refrac',
                                                        'i_offset'))
        sim.initialize(self._cell, 'v', self._cell[0].v_rest)

    def start_record_voltage(self):
        """
        Records the voltage of the neuron
        """
        self._cell.record_v()

    def _update_parameters(self, params):
        """
        Makes sure that all relevant parameters for connecting the device to a neuron population
        are set. If not explicit connector is provided as parameter it is created from default
        values.

        :param params: The parameter dictionary
        """
        super(PyNNLeakyIntegrator, self)._update_parameters(params)

        if not "connector" in self._parameters or not self._parameters["connector"]:
            weights = self._parameters["weights"]
            if not weights:
                weights = self._get_connector_weight()
            delays = self._parameters["delays"]
            self._parameters["connector"] = sim.AllToAllConnector(weights=weights, delays=delays)
        else:
            conn = self._parameters["connector"]
            if isinstance(conn, dict):
                weights = self._parameters["weights"]
                if not weights:
                    weights = conn["weights"]
                if not weights:
                    weights = self._get_connector_weight()
                delays = self._parameters["delays"]
                if conn["mode"] == "OneToOne":
                    self._parameters["connector"] = \
                        sim.OneToOneConnector(weights=weights, delays=delays)
                elif conn["mode"] == "AllToAll":
                    self._parameters["connector"] = \
                        sim.AllToAllConnector(weights=weights, delays=delays)
                elif conn["mode"] == "Fixed":
                    self._parameters["connector"] = \
                        sim.FixedNumberPreConnector(conn["n"], weights, delays)
                else:
                    raise Exception("Invalid connector mode")
        if isinstance(self._parameters["synapse_dynamics"], dict):
            dyn = self._parameters["synapse_dynamics"]
            if dyn["type"] == "TsodyksMarkram":
                self._parameters["synapse_dynamics"] = \
                    sim.SynapseDynamics(sim.TsodyksMarkramMechanism(
                        U=dyn["U"], tau_rec=dyn["tau_rec"], tau_facil=dyn["tau_facil"]))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """

        return sim.Projection(presynaptic_population=neurons,
                              postsynaptic_population=self._cell,
                              **self.get_parameters(("method", "connector"),
                                                     "source",
                                                     "target",
                                                     "synapse_dynamics",
                                                     "label",
                                                     "rng"))

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        self._voltage = self._cell.get_v()[-1, -1]
