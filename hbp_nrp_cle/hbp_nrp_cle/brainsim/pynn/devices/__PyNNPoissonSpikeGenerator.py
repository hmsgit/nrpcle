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
from hbp_nrp_cle.brainsim.pynn.devices.__PyNNDictParser import set_synapse_type, set_connector

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
        "rng": None,
        "n": 1
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
        self._last_rate_before_deactivation = None
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

    def _activate(self):
        """Activate this source, if inactive, restoring the previous rate value"""
        if not self.active:
            self.rate = self._last_rate_before_deactivation

    def _deactivate(self):
        """Deactivate this source, if active, setting the rate to zero"""
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
        Create Poisson spike generator device
        """
        self._generator = self.sim()\
            .Population(self._parameters["n"],
                        self.sim().SpikeSourcePoisson(
                            **self.get_parameters("duration",
                                                  "start",
                                                  "rate")
                        ),
                        label=self._parameters["label"])

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
        set_synapse_type(self._parameters, self.sim())
        set_connector(self._parameters, self.sim(), params)

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
        return self.sim().Projection(presynaptic_population=self._generator,
                                     postsynaptic_population=neurons,
                                     **self.get_parameters("source",
                                                           "receptor_type",
                                                           "connector",
                                                           "synapse_type"))

    def _disconnect(self):
        """
        Disconnects the device by setting rate to 0 since we cannot delete the device or
        connection directly via PyNN.
        """
        if self._generator:
            self.rate = 0.0
            self._generator = None
