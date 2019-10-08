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
This module contains the shared codebase for all NEST-specific implementation of leaky integrator
devices.
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IBrainDevice
from hbp_nrp_cle.brainsim.nest.devices.__NestDictParser import set_synapse_type, set_connector

import nest


class NestLeakyIntegrator(AbstractBrainDevice, IBrainDevice):
    """
    Abstract super class of brain devices that represent neuronal spike activity by reading out
    the membrane potential of additional integrate and fire neurons. The concrete type of neuron
    depends on the concrete realization of this class.
    """

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'V_th' is set to infinity
        by default in order to forbid the neuron to elicit spikes.

        :param V_th: Threshold voltage
        :param C_m: Membrane capacitance
        :param tau_m: Membrane time constant
        :param tau_syn_ex: Excitatory synaptic time constant
        :param tau_syn_in: Inhibitory synaptic time constant
        :param E_L: Resting potential
        :param V_reset: Reset potential
        :param t_ref: Refractory time constant
        :param I_e: Offset current
        :param connector: a connector type, or, if neurons is
            a list of two populations, a list of two connectors
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory']
        :param synapse_type: a synapse type
        """
        super(NestLeakyIntegrator, self).__init__(**params)

        self._cell = None
        self._voltage = 0.0
        self._connected = False

        self.create_device()

    @staticmethod
    def _get_cell_type():
        """
        Returns the cell type of the neuron to be created by this device

        :return: a NEST cell type
        """
        raise NotImplementedError("The cell type has to be specified in a concrete implementation")

    @property
    def voltage(self):
        """
        Returns the membrane voltage of the cell
        """
        return self._voltage

    def sim(self):  # pragma: no cover
        """
        Gets the simulator module to use
        """
        raise NotImplementedError("This method must be overridden in a derived class")

    def create_device(self):
        """
        Creates a LIF neuron with alpha-shaped post synaptic currents
        and current-based synapses
        """
        self._cell = nest.Create(self._get_cell_type(),
                                 1,
                                 self.get_parameters('V_th',
                                                     'C_m',
                                                     'tau_m',
                                                     'tau_syn_ex',
                                                     'tau_syn_in',
                                                     'E_L',
                                                     'V_reset',
                                                     't_ref',
                                                     'I_e'))
        v = self._parameters["E_L"]
        nest.SetStatus(self._cell, {'V_m': v})
        self._voltage = v

    def _update_parameters(self, params):
        """
        Makes sure that all relevant parameters for connecting the device to a neuron population
        are set. If not explicit connector is provided as parameter it is created from default
        values.

        :param params: The parameter dictionary
        """
        super(NestLeakyIntegrator, self)._update_parameters(params)
        set_connector(self._parameters, params)
        set_synapse_type(self._parameters)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        syn_dict = self._parameters["synapse_type"].copy()
        syn_dict["weight"] = self._parameters["weight"]
        syn_dict["delay"] = self._parameters["delay"]
        nest.Connect(neurons,
                     self._cell,
                     conn_spec=self.get_parameters("connector")["connector"],
                     syn_spec=syn_dict)
        self._connected = True

    def _disconnect(self):
        """
        Disconnects the leaky integrator by disabling recording, no new values
        will be produced.
        """
        self._connected = False

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        if self._connected:
            self._voltage = nest.GetStatus(self._cell, 'V_m')[0]
