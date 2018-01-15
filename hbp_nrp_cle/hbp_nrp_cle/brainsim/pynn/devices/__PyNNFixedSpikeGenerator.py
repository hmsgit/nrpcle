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
'''
Implementation of PyNNFixedSpikeGenerator
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator
from hbp_nrp_cle.brainsim.pynn.devices.__SynapseTypes import set_synapse_type
from pyNN.random import RandomDistribution
import numpy as np

__author__ = 'Dimitri Probst, Sebastian Krach, Georg Hinkel'


class PyNNFixedSpikeGenerator(AbstractBrainDevice, IFixedSpikeGenerator):
    """
    Represents a spike generator which generated equidistant
    spike times at a given frequency
    """

    default_parameters = {
        'initial_rate': 0.0,
        'cm': 1.0,
        'tau_m': 1000.0,
        'tau_refrac': 0.2,
        'v_thresh': -50.0,
        'v_reset': -100.0,
        'v_rest': -100.0,
        'connector': None,
        'weight': None,
        'delay': None,
        'source': None,
        'receptor_type': 'excitatory',
        'synapse_type': None,
        'label': None
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Fixed spike generator.

        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        :param connector: a PyNN Connector object, or, if neurons is
            a list of two populations, a list of two Connector objects
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_type: a PyNN Synapse Type object
        :param label: label of the Projection object
        :param rng: RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNFixedSpikeGenerator, self).__init__(**params)

        self._generator = None
        self._currentsource = None

        self._calculate_rate_and_current = self._setup_rate_and_current_calculation()

        (self._rate,
         self._current) = self._calculate_rate_and_current(self._parameters["initial_rate"])

        self._last_rate_before_deactivation = None

        self.create_device()

    @property
    def rate(self):
        """
        Returns the frequency of the Fixed spike generator
        """
        return self._rate

    @rate.setter
    def rate(self, value):  # pragma: no cover
        """
        Sets the frequency of the Fixed spike generator

        :param value: float
        """
        self._rate, current = self._calculate_rate_and_current(value)

        if current != self._current:
            self._current = current
            self._currentsource.set(amplitude=current)

    def _activate(self):
        """
        Activates the Fixed spike generator restoring the last value of the rate
        """
        if not self.active:
            self.rate = self._last_rate_before_deactivation

    def _deactivate(self):
        """
        Deactivates the Fixed spike generator setting the rate to zero
        """
        if self.active:
            self._last_rate_before_deactivation = self.rate
            self.rate = 0.0

    def sim(self):  # pragma: no cover
        """
        Gets the simulator module to use
        """
        raise NotImplementedError("This method must be overridden in a derived class")

    def create_device(self):
        """
        Create a fixed spike-distance device
        """

        self._generator = self.sim().Population(1, self.sim().IF_curr_exp(
            **self.get_parameters("cm",
                                  "tau_m",
                                  "v_thresh",
                                  "v_reset",
                                  "v_rest")))
        self.sim().initialize(self._generator, v=self._generator[0].v_rest)

        self._currentsource = self.sim().DCSource(amplitude=self._current)
        self._currentsource.inject_into(self._generator)

    def _setup_rate_and_current_calculation(self):
        """
        This method sets up the calculation of the suitable current based on specified spiking
        rate values. As this calculation is dependent on neuron parameters which are only set
        once, this method returns a callable which expects the desired spiking rate and returns a
        tuple of closest achievable rate and the appropriate current in nA.

        :return: a callable function: float --> (float, float)
        """
        tau_m, tau_refrac, cm, v_thresh, v_rest = \
            self.get_parameters("tau_m", "tau_refrac", "cm", "v_thresh", "v_rest").values()

        def calculate_rate_and_current(rate):
            """
            Returns current in nA corresponding to frequency "rate". If the specified spiking rate
            is not achievable due to the neuron configuration the current is determined for the
            possible maximum.

            :param rate: Frequency in Hz
            :return: The frequency which results from injecting the configured neuron with the
                    resulting current in Hz.
            :return: The resulting dc current
            """
            nom = (cm / tau_m) * (v_thresh - v_rest)
            denom = 1.0
            if rate != 0.0:
                inter_time = 1000.0 / rate
                if inter_time < 10 * tau_refrac:
                    rate = 100.0 / tau_refrac
                    inter_time = 10 * tau_refrac
                denom = 1.0 - np.exp((tau_refrac - inter_time) / tau_m)
            return rate, nom / denom

        return calculate_rate_and_current

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weight and delay between the neurons and the device
        are sampled from a uniform distribution.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        # As connection parameters depend on the passed neuron structure the parameters are
        # evaluated in the connect method.
        weight = self._parameters["weight"]

        if not weight:
            weight = self._get_default_weights(neurons.conductance_based)
            self._parameters["weight"] = weight

        set_synapse_type(self._parameters, self.sim())

        return self.sim().Projection(presynaptic_population=self._generator,
                                     postsynaptic_population=neurons,
                                     **self.get_parameters("connector",
                                                           "source",
                                                           "receptor_type",
                                                           "synapse_type",
                                                           "label"))

    def _disconnect(self):
        """
        Disconnects the device by setting rate to 0 since we cannot delete the device or
        connection directly via PyNN.
        """
        if self._currentsource:
            self.rate = 0.0
            self._generator = None
            self._currentsource = None

    def _update_parameters(self, params):
        """
        This method updates the device parameter dictionary with the provided parameter
        dictionary. The dictionary has to be validated before as this method assumes it to be
        correct.

        Overriding subclasses can provide additional configuration parameter adaption as this
        method is called before the brain simulator devices are constructed.

        :param params: The validated parameter dictionary
        """
        super(PyNNFixedSpikeGenerator, self)._update_parameters(params)

        if "connector" not in self._parameters or not self._parameters["connector"]:
            self._parameters["connector"] = self.sim().AllToAllConnector()
        else:
            conn = self._parameters["connector"]
            if isinstance(conn, dict):
                if "weight" not in params:
                    self._parameters["weight"] = conn.get("weight")
                if "delay" not in params:
                    self._parameters["delay"] = conn.get("delay")
                if conn.get("mode") == "OneToOne":
                    self._parameters["connector"] = \
                        self.sim().OneToOneConnector()
                elif conn.get("mode") == "AllToAll":
                    self._parameters["connector"] = \
                        self.sim().AllToAllConnector()
                elif conn.get("mode") == "Fixed":
                    self._parameters["connector"] = \
                        self.sim().FixedNumberPreConnector(conn.get("n", 1))
                else:
                    raise Exception("Invalid connector mode")

    def _get_default_weights(self, conductance_based):
        """
        Gets the default weights distribution

        :param conductance_based: Indicates whether the connected neurons
        are conductance based
        """
        if self._parameters.get("target") == 'excitatory':
            weights = RandomDistribution('uniform', [0.0, 0.01])
        else:
            if conductance_based:
                weights = RandomDistribution('uniform', [0.0, 0.01])
            else:
                weights = RandomDistribution('uniform', [-0.01, -0.0])
        return weights
