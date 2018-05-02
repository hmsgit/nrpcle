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

from hbp_nrp_cle.brainsim.pynn.devices import PyNNPoissonSpikeGenerator
from hbp_nrp_cle.brainsim.pynn_spiNNaker import spynnaker as sim
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connection
import logging
import decimal


logger = logging.getLogger(__name__)


__author__ = 'Georg Hinkel'


class PyNNSpiNNakerPoissonSpikeGenerator(PyNNPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    __counter = 0

    def __init__(self, **config):
        super(PyNNSpiNNakerPoissonSpikeGenerator, self).__init__(**config)
        self.__connection = None

    default_parameters = {
        "duration": 1.0e10,
        "start": 0.0,
        "rate": 0.0,
        "connector": None,
        "weight": 0.00015,
        "delay": 1.0,
        "source": None,
        "receptor_type": "excitatory",
        "synapse_type": None,
        "label": None,
        "rng": None,
        "n": 1
    }

    def sim(self):
        """
        Gets the simulator module to use
        """
        return sim

    def __init_connection(self, pop_label, connection):
        """
        Gets called when the spike connection is ready

        :param pop_label: The population label of the generator
        :param connection: The poisson connection
        """
        # Spinnaker currently adds a control label suffix
        # We override the label setting here to reflect that in a hopefully robust manner
        self._parameters["label"] = pop_label

        if self.__connection is None:
            logger.info("Poisson Spike generator ready to reconfigure rates")
        elif self.__connection is not connection:
            raise Exception("Poisson spike generator already has a connection assigned")
        self.__connection = connection

    def _update_parameters(self, params):
        super(PyNNSpiNNakerPoissonSpikeGenerator, self)._update_parameters(params)
        if self._parameters["label"] is None:
            self._parameters["label"] = "PoissonGenerator{}"\
                .format(PyNNSpiNNakerPoissonSpikeGenerator.__counter)
            PyNNSpiNNakerPoissonSpikeGenerator.__counter += 1

    def connect(self, neurons):
        sim.external_devices.add_poisson_live_rate_control(
            self._generator, receive_port=live_connection.POISSON_PORT
        )
        live_connection.register_poisson(self._generator.label, self.__init_connection)
        return super(PyNNSpiNNakerPoissonSpikeGenerator, self).connect(neurons)

    @property
    def rate(self):
        """
        Returns the frequency of the Poisson spike generator
        """
        return self._parameters["rate"]

    # pylint: disable=arguments-differ
    @rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Poisson spike generator

        :param value: float
        """
        if self.__connection is None:
            # try to reconfigure using the PyNN interface
            logger.warning("Poisson rate update connection not yet available, "
                           "reconfiguring Poisson generator using PyNN")
            self._generator.set(rate=value)
        else:
            # Spinnaker multiplies the rate with a decimal, so we have to convert the rate
            val = decimal.Decimal(value)
            self.__connection.set_rates(self._parameters["label"],
                                        [(i, val) for i in range(self._parameters["n"])])
        self._parameters["rate"] = value
