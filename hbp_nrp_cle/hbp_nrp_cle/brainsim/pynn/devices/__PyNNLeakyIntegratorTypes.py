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
This module contains the PyNN-specific implementation of the ILeakyIntegratorAlpha and the
ILeakyIntegratorExp device.
'''

from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, ILeakyIntegratorExp
from .__PyNNLeakyIntegrator import PyNNLeakyIntegrator
from pyNN.random import RandomDistribution

__author__ = "Sebastian Krach"

# pylint: disable=abstract-method


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
        'weight': None,
        'delay': 0.1,
        'source': None,
        'receptor_type': 'excitatory',
        'synapse_type': None,
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
        :param weight: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified weights. Default: 0.01 (excitatory) or -0.01 (else)
        :param delay: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified delay. Default: 0.1
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: "excitatory" or "inhibitory". Default: "excitatory"
        :param synapse_type: (optional) a PyNN Synapse Type object
        :param label: (optional) label of the Projection object
        :param rng: (optional) RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNLeakyIntegratorAlpha, self).__init__(**params)

    @staticmethod
    def _get_cell_type(sim, **params):
        """
        Returns the cell type of the neuron to be created by this device. In this case the neuron
        to be created adheres to PyNN's "Leaky integrate and fire model with fixed threshold and
        alpha-function-shaped post-synaptic current"

        :return: PyNN IF_curr_alpha cell type
        """
        return sim.IF_curr_alpha(**params)

    def _get_connector_weight(self):
        """
        Returns the default connector weight in case no explicit weight is specified as parameter

        :return: the weight of the synaptic connection
        """
        if self._parameters["receptor_type"] == 'excitatory':
            return 0.01
        else:
            return -0.01


class PyNNLeakyIntegratorExp(PyNNLeakyIntegrator, ILeakyIntegratorExp):
    """
    Represents the membrane potential of a current-based LIF neuron
    with decaying-exponential post-synaptic currents.
    """

    default_parameters = {
        'v_thresh': float('inf'),
        'cm': 1.0,
        'tau_m': 20.0,
        'tau_syn_E': .5,
        'tau_syn_I': .5,
        'v_rest': 0.0,
        'v_reset': 0.0,
        'tau_refrac': 0.1,
        'i_offset': 0.0,
        'connector': None,
        'weight': None,
        'delay': RandomDistribution('uniform', [0.1, 2.0]),
        'source': None,
        'receptor_type': 'excitatory',
        'synapse_type': None,
        'label': None,
        'rng': None
    }

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
        :param weight: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified weights.
                Default: randomly distributed in [0, 0.01] (excitatory) or [-0.01, 0] (else)
        :param delay: if no preconfigured connector is specified the default AllToAllConnector
                is configured with the specified delay.
                Default: randomly distributed in [0.1, 2.0]
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: "excitatory" or "inhibitory". Default: "excitatory"
        :param synapse_type: (optional) a PyNN Synapse Type object
        :param label: (optional) label of the Projection object
        :param rng: (optional) RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNLeakyIntegratorExp, self).__init__(**params)

    @staticmethod
    def _get_cell_type(sim, **params):
        """
        Returns the cell type of the neuron to be created by this device. In this case the neuron
        to be created adheres to PyNN's "Leaky integrate and fire model with fixed threshold and
        decaying-exponential post-synaptic current"

        :return: PyNN IF_curr_exp cell type
        """
        return sim.IF_curr_exp(**params)

    def _get_connector_weight(self):
        """
        Returns the default connector weight in case no explicit weight is specified as parameter

        :return: the weight of the synaptic connection
        """
        if self._parameters["receptor_type"] == 'excitatory':
            return RandomDistribution('uniform', [0., 0.01])
        else:
            return RandomDistribution('uniform', [-0.01, -0.])
