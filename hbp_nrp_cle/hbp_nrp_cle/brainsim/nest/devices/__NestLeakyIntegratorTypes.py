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
This module contains the NEST-specific implementation of the ILeakyIntegratorAlpha and the
ILeakyIntegratorExp device.
"""

from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, ILeakyIntegratorExp
from .__NestLeakyIntegrator import NestLeakyIntegrator


__author__ = 'LorenzoVannucci'


# pylint: disable=abstract-method
class NestLeakyIntegratorAlpha(NestLeakyIntegrator, ILeakyIntegratorAlpha):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    default_parameters = {
        'V_th': float('inf'),
        'C_m': 1000.0,
        'tau_m': 10.0,
        'tau_syn_ex': 2.,
        'tau_syn_in': 2.,
        'E_L': 0.0,
        'V_reset': 0.0,
        't_ref': 0.1,
        'I_e': 0.0,
        'connector': None,
        'weight': 10.0,
        'delay': 0.1,
        'receptor_type': 'excitatory',
        'synapse_type': None,
    }

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
        super(NestLeakyIntegratorAlpha, self).__init__(**params)

    @staticmethod
    def _get_cell_type():
        """
        Returns the cell type of the neuron to be created by this device. In this case the neuron
        to be created adheres to NEST's "Leaky integrate-and-fire model with alpha-function shaped
        synaptic currents"

        :return: NEST iaf_psc_alpha cell type
        """
        return 'iaf_psc_alpha'


class NestLeakyIntegratorExp(NestLeakyIntegrator, ILeakyIntegratorExp):
    """
    Represents the membrane potential of a current-based LIF neuron
    with decaying-exponential post-synaptic currents.
    """

    default_parameters = {
        'V_th': float('inf'),
        'C_m': 1000.0,
        'tau_m': 20.0,
        'tau_syn_ex': .5,
        'tau_syn_in': .5,
        'E_L': 0.0,
        'V_reset': 0.0,
        't_ref': 0.1,
        'I_e': 0.0,
        'connector': None,
        'weight': 10.0,
        'delay': {'distribution': 'uniform', 'low': 0.1, 'high': 2.0},
        'receptor_type': 'excitatory',
        'synapse_type': None,
    }

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
        super(NestLeakyIntegratorExp, self).__init__(**params)

    @staticmethod
    def _get_cell_type():
        """
        Returns the cell type of the neuron to be created by this device. In this case the neuron
        to be created adheres to NEST's "Leaky integrate-and-fire model with exponential shaped
        synaptic currents"

        :return: NEST iaf_psc_exp cell type
        """
        return 'iaf_psc_exp'
