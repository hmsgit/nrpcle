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
Implementation of PyNNLeakyIntegratorAlpha
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNLeakyIntegrator
from hbp_nrp_cle.brainsim.pynn.devices import PyNNLeakyIntegratorAlpha, PyNNLeakyIntegratorExp
import nest

__author__ = 'DimitriProbst'


# PyLint does not recognize the following class as abstract as it does not define new methods
# raising NotImplementedException
# pylint: disable=abstract-method
class PyNNNestLeakyIntegrator(PyNNLeakyIntegrator):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    def start_record_voltage(self):
        # Since we get the data directly from Nest and Nest supports reading of just the latest
        # value we don't need to record the entire voltage trace.
        pass

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """

        self._voltage = nest.GetStatus([self._cell[0]])[0]['V_m']


class PyNNNestLeakyIntegratorAlpha(PyNNNestLeakyIntegrator, PyNNLeakyIntegratorAlpha):
    """
    The Nest-specific adaption of the PyNN leaky Integrator using a current-based LIF neuron with
    alpha-shaped post synaptic currents
    """


class PyNNNestLeakyIntegratorExp(PyNNNestLeakyIntegrator, PyNNLeakyIntegratorExp):
    """
    The Nest-specific adaption of the PyNN leaky Integrator using a current-based LIF neuron with
    decaying-exponential post-synaptic currents
    """
