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
Implementation of PyNNLeakyIntegratorExp
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNLeakyIntegratorExp
from hbp_nrp_cle.brainsim.pynn_spiNNaker import spynnaker as sim

__author__ = 'DimitriProbst, Felix Schneider'


class PyNNSpiNNakerLeakyIntegratorExp(PyNNLeakyIntegratorExp):
    """
    Represents the membrane potential of a current-based LIF neuron
    with decaying-exponential post-synaptic currents
    """

    def create_device(self, **params):
        """
        Creates a LIF neuron with decaying-exponential post-synaptic currents
        and current-based synapses

        :param params: Dictionary of neuron configuration parameters
        :param v_thresh: Threshold voltage , default: infinity
        :param cm: Membrane capacitance, default: 1.0 nF
        :param tau_m: Membrane time constant, default: 20.0 ms
        :param tau_syn_E: Excitatory synaptic time constant, default: 0.5 ms
        :param tau_syn_I: Inhibitory synaptic time constant, default: 0.5 ms
        :param v_rest: Resting potential, default: 0.0 mV
        :param v_reset: Reset potential, default: 0.0 mV
        :param tau_refrac: Refractory time constant, default: 0.1 ms
        :param i_offset: Offset current, default: 0.0 nA
        """
        # SpiNNaker does not support infinite spike potential
        cellparams = {'v_thresh': params.get('v_thresh', 30000),
                      'cm': params.get('cm', 1.0),
                      'tau_m': params.get('tau_m', 10.0),
                      'tau_syn_E': params.get('tau_syn_E', 2.0),
                      'tau_syn_I': params.get('tau_syn_I', 2.0),
                      'v_rest': params.get('v_rest', 0.0),
                      'v_reset': params.get('v_reset', 0.0),
                      'tau_refrac': params.get('tau_refrac', 0.1),
                      'i_offset': params.get('i_offset', 0.0)}
        self._cell = sim.Population(1, sim.IF_curr_exp, cellparams)
        self._cell.set(v=self._cell.get('v_rest'))

    def sim(self):
        """
        Gets the simulator module to use
        """
        return sim

    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        self._voltage = self._cell.spinnaker_get_data('v')[-1, -1]
