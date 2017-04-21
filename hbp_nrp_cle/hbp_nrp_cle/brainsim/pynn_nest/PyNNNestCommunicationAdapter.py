# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

import logging
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, \
    IDCSource, IACSource, INCSource, ILeakyIntegratorExp, IPopulationRate, \
    IFixedSpikeGenerator, ISpikeRecorder

from hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter import PyNNCommunicationAdapter
from .devices import PyNNNestACSource, PyNNNestDCSource, PyNNNestNCSource, \
    PyNNNestLeakyIntegratorAlpha, PyNNNestLeakyIntegratorExp, PyNNNestFixedSpikeGenerator, \
    PyNNNestPopulationRate, PyNNNestSpikeRecorder

logger = logging.getLogger(__name__)

__author__ = 'Dimitri Probst, Sebastian Krach'


# Pylint for some reason does not see that the methods is_population and create_view are already
# overridden
# pylint: disable=abstract-method
class PyNNNestCommunicationAdapter(PyNNCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IFixedSpikeGenerator: PyNNNestFixedSpikeGenerator,
                     IDCSource: PyNNNestDCSource,
                     IACSource: PyNNNestACSource,
                     INCSource: PyNNNestNCSource,
                     ILeakyIntegratorAlpha: PyNNNestLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: PyNNNestLeakyIntegratorExp,
                     IPopulationRate: PyNNNestPopulationRate,
                     ISpikeRecorder: PyNNNestSpikeRecorder}

    def initialize(self):
        """
        Marks the PyNN adapter as initialized
        """
        super(PyNNNestCommunicationAdapter, self).initialize()

    def _get_device_type(self, device_type):
        """
        Returns the pynn specific implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete type
        """

        if device_type in self.__device_dict:
            return self.__device_dict[device_type]
        return super(PyNNNestCommunicationAdapter, self)._get_device_type(device_type)
