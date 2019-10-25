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
Implementation of NestPoissonSpikeGenerator
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IPoissonSpikeGenerator
from hbp_nrp_cle.brainsim.nest.devices.__NestDictParser import set_synapse_type, set_connector

import nest

__author__ = 'LorenzoVannucci'


class NestPoissonSpikeGenerator(AbstractBrainDevice, IPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    default_parameters = {
        "stop": float('inf'),
        "start": 0.0,
        "rate": 0.0,
        "connector": None,
        "weight": 0.15,
        "delay": 0.1,
        "receptor_type": "excitatory",
        "synapse_type": None,
        "n": 1
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Poisson spike generator.

        :param start: Start time of spike train, default: 0.0 ms
        :param stop: Stop time of spike train, default: infinity
        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        :param connector: a connector type, or, if neurons is
            a list of two populations, a list of two connectors
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory']
        :param synapse_type: a synapse type
        """
        super(NestPoissonSpikeGenerator, self).__init__(**params)

        self._generator = None
        self._last_rate_before_deactivation = None
        self.create_device()

    @property
    def rate(self):
        """
        Returns the frequency of the Poisson spike generator
        """
        return nest.GetStatus(self._generator, 'rate')[0]

    @rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Poisson spike generator

        :param value: float
        """
        nest.SetStatus(self._generator, {'rate': value})

    def _activate(self):
        """Activate this source, if inactive, restoring the previous rate value"""
        if not self.active:
            self.rate = self._last_rate_before_deactivation

    def _deactivate(self):
        """Deactivate this source, if active, setting the rate to zero"""
        if self.active:
            self._last_rate_before_deactivation = self.rate
            self.rate = 0.0

    def create_device(self):
        """
        Create Poisson spike generator device
        """
        self._generator = nest.Create('poisson_generator',
                                      self._parameters["n"],
                                      self.get_parameters("stop",
                                                          "start",
                                                          "rate"))

    def _update_parameters(self, params):
        """
        This method updates the device parameter dictionary with the provided parameter
        dictionary. The dictionary has to be validated before as this method assumes it to be
        correct.

        Overriding subclasses can provide additional configuration parameter adaption as this
        method is called before the brain simulator devices are constructed.

        :param params: The validated parameter dictionary
        """
        super(NestPoissonSpikeGenerator, self)._update_parameters(params)
        set_synapse_type(self._parameters)
        set_connector(self._parameters, params)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        syn_dict = self._parameters["synapse_type"].copy()
        syn_dict["weight"] = self._parameters["weight"]
        syn_dict["delay"] = self._parameters["delay"]
        nest.Connect(self._generator,
                     neurons,
                     conn_spec=self.get_parameters("connector")["connector"],
                     syn_spec=syn_dict)

    def _disconnect(self):
        """
        Disconnects the device by setting rate to 0 since we cannot delete the device or
        connection directly via Nest.
        """
        if self._generator:
            self.rate = 0.0
            self._generator = None
