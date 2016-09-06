'''
This module contains the shared codebase for all PyNN-specific implementation of leaky integrator
devices.
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.pynn import simulator as sim
from hbp_nrp_cle.brainsim.pynn.devices.__SynapseTypes import set_synapse_type

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
        :param synapse_type: a PyNN Synapse Type object
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
    def _get_cell_type(**params):
        """
        Returns the cell type of the neuron to be created by this device
        :param params: parameters passed to the cell type constructor
        :return: a PyNN cell type
        """
        raise NotImplementedError("The cell type has to be specified in a concrete implementation")

    def _get_connector_weight(self):  # pragma: no cover
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
                                    self._get_cell_type(**self.get_parameters('v_thresh',
                                                                              'cm',
                                                                              'tau_m',
                                                                              'tau_syn_E',
                                                                              'tau_syn_I',
                                                                              'v_rest',
                                                                              'v_reset',
                                                                              'tau_refrac',
                                                                              'i_offset')))
        sim.initialize(self._cell, v=self._parameters["v_rest"])

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

        if "connector" not in self._parameters or not self._parameters["connector"]:
            self._parameters["connector"] = sim.AllToAllConnector()
        else:
            conn = self._parameters["connector"]
            if isinstance(conn, dict):
                self.__apply_connector(conn, params)
        if not self._parameters["weight"]:
            self._parameters["weight"] = self._get_connector_weight()

        set_synapse_type(self._parameters, sim)

    def __apply_connector(self, conn, params):
        """
        Applies the given connector respecting the given manual parameters

        :param conn: The connector dict
        :param params: Manual parameters
        """
        weights = conn.get("weight")
        if weights and "weight" not in params:
            self._parameters["weight"] = weights
        delays = conn.get("delay")
        if delays and "delay" not in params:
            self._parameters["delay"] = delays

        conn_mode = conn.get("mode")
        if conn_mode == "OneToOne":
            self._parameters["connector"] = \
                sim.OneToOneConnector()
        elif conn_mode == "AllToAll":
            self._parameters["connector"] = \
                sim.AllToAllConnector()
        elif conn_mode == "Fixed":
            self._parameters["connector"] = \
                sim.FixedNumberPreConnector(conn.get("n", 1))
        else:
            raise Exception("Invalid connector mode")

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
                              **self.get_parameters("connector",
                                                    "source",
                                                    "receptor_type",
                                                    "synapse_type",
                                                    "label"))

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        self._voltage = self._cell.get_data('v', clear=True)[-1, -1]
