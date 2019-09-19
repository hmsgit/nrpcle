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
Implementation of PyNNPopulationRate
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNPopulationRate
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice

import pyNN.nest as nestsim
from mpi4py import MPI

from scipy.integrate import simps
import numpy as np

__author__ = 'DimitriProbst'


class PyNNNestPopulationRate(PyNNPopulationRate, PyNNNestDevice):
    """
    Represents the rate of a population of LIF neurons by
    measuring and normalizing the membrane potential of a
    leaky integrator with decaying-exponential post-synaptic currents
    """

    def sim(self):
        """
        Gets the simulator module to use
        """
        return nestsim

    def _start_record_rate(self):
        # Since we get the data directly from Nest and Nest supports reading of just the latest
        # value we don't need to record the entire voltage trace.
        pass

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the rate value

        :param time: The current simulation time
        """

        # single process, direct access to voltage
        if not self.mpi_aware:
            self._rate = self.GetStatus([self._cell[0]])[0]['V_m']

        # multi-process, gather the voltage from all nodes, CLE is guaranteed to be rank 0
        else:
            data = self.GetStatus([self._cell[0]])[0]
            values = MPI.COMM_WORLD.gather(data['V_m'] if 'V_m' in data else 0.0, root=0)

            # only let the CLE continue processing
            if MPI.COMM_WORLD.Get_rank() > 0:
                return

            # only one process will have the neuron and voltage accessible
            self._rate = sum(values)

    def _create_device(self):
        """
        Creates a LIF neuron with decaying-exponential post-synaptic currents
        and current-based synapses.
        """

        self._cell = self.sim().Population(1, self.sim().IF_curr_exp(
            **self.get_parameters(("tau_m", "tau_fall"),
                                  ("tau_syn_E", "tau_rise"),
                                  "v_thresh",
                                  "cm",
                                  "v_rest")))


        #self.sim().initialize(self._cell, v=self._cell[0].v_rest)
        params = self.get_parameters("v_rest")
        self.sim().initialize(self._cell, v=params["v_rest"])

    def _calculate_weight(self):
        """
        Calculates the weight of a neuron from the population to the device
        such that the area below the resulting PSP is 1. The exact shape of a
        PSP can be found e.g. in Bytschok, I., Diploma thesis.
        """
        params = self.get_parameters("tau_rise","tau_fall","cm")
        tau_c = (1. / params["tau_rise"] - 1. / params["tau_fall"]) ** -1
        t_end = -np.log(1e-10) * params["tau_fall"]
        x_new = np.arange(0., t_end, 0.1)
        y_new = tau_c / params["cm"] * (np.exp(
                -x_new / params["tau_fall"]) - np.exp(
                -x_new / params["tau_rise"]))
        self._weight = 1.0 / simps(y_new, dx=self.sim().get_time_step())


