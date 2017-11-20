# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
Implementation of PyNNPoissonSpikeGenerator
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IPoissonSpikeGenerator
from hbp_nrp_cle.brainsim.pynn.devices.__SynapseTypes import set_synapse_type

__author__ = 'Dimitri Probst, Georg Hinkel'


class PyNNPoissonSpikeGenerator(AbstractBrainDevice, IPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    default_parameters = {
        "duration": float("inf"),
        "start": 0.0,
        "rate": 0.0,
        "connector": None,
        "weight": 0.00015,
        "delay": 0.1,
        "source": None,
        "receptor_type": "excitatory",
        "synapse_type": None,
        "label": None,
        "rng": None
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Poisson spike generator.

        :param duration: Duration of spike train, default: infinity
        :param start: Start time of spike train, default: 0.0 ms
        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        :param connector: a PyNN Connector object, or, if neurons is
            a list of two populations, a list of two Connector objects
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_dynamics: a PyNN SynapseDy
        :param label: label of the Projection object
        :param rng: RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNPoissonSpikeGenerator, self).__init__(**params)

        self._generator = None

        self.create_device()

    @property
    def rate(self):
        """
        Returns the frequency of the Poisson spike generator
        """
        return self._generator.get('rate')[0]

    @rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Poisson spike generator

        :param value: float
        """
        self._generator.set(rate=value)

    def sim(self):  # pragma: no cover
        """
        Gets the simulator module to use
        """
        raise NotImplementedError("This method must be overridden in a derived class")

    def create_device(self):
        """
        Create Poisson spike generator device
        """
        self._generator = self.sim().Population(1, self.sim().SpikeSourcePoisson(
                **self.get_parameters("duration",
                                      "start",
                                      "rate")))

    def _update_parameters(self, params):
        """
        This method updates the device parameter dictionary with the provided parameter
        dictionary. The dictionary has to be validated before as this method assumes it to be
        correct.

        Overriding subclasses can provide additional configuration parameter adaption as this
        method is called before the brain simulator devices are constructed.

        :param params: The validated parameter dictionary
        """
        super(PyNNPoissonSpikeGenerator, self)._update_parameters(params)

        weights = params.get("weight")
        delays = params.get("delay")

        if "connector" in params:
            conn = self._parameters["connector"]
            if isinstance(conn, dict):
                delays, weights = self.__apply_connector(conn, delays, weights)
        else:
            self._parameters["connector"] = self.sim().AllToAllConnector()
        if weights:
            self._parameters["weight"] = weights
        if delays:
            self._parameters["delay"] = delays

        set_synapse_type(self._parameters, self.sim())

    def __apply_connector(self, conn, delays, weights):
        """
        Applies the given connector dictionary

        :param conn: The connector as dictionary
        :param delays: The current delays
        :param weights: The current weights
        :returns: The updated weights and delays
        """
        if not weights:
            weights = conn.get("weight")
        if not delays:
            delays = conn.get("delay")
        conn_mode = conn.get("mode")
        if conn_mode == "OneToOne":
            self._parameters["connector"] = \
                self.sim().OneToOneConnector()
        elif conn_mode == "AllToAll":
            self._parameters["connector"] = \
                self.sim().AllToAllConnector()
        elif conn_mode == "Fixed":
            self._parameters["connector"] = \
                self.sim().FixedNumberPreConnector(conn.get("n", 1))
        else:
            raise Exception("Invalid connector mode")
        return delays, weights

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

        if not self._parameters["synapse_type"]:
            if not (self._parameters["receptor_type"] == 'excitatory' or neurons.conductance_based):
                self._parameters["weight"] = -abs(self._parameters["weight"])

            self._parameters["synapse_type"] = self.sim().StaticSynapse(
                **self.get_parameters("weight", "delay"))

        return self.sim().Projection(presynaptic_population=self._generator,
                                     postsynaptic_population=neurons,
                                     **self.get_parameters("source",
                                                           "receptor_type",
                                                           "connector",
                                                           "synapse_type",
                                                           "label"))

    def _disconnect(self):
        """
        Disconnects the device by setting rate to 0 since we cannot delete the device or
        connection directly via PyNN.
        """
        if self._generator:
            self.rate = 0.0
            self._generator = None
