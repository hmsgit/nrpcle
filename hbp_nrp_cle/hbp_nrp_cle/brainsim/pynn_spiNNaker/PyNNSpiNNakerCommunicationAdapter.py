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
PyNNSpiNNakerCommunicationAdapter.py
moduleauthor: fschneid@fzi.de
'''

import logging
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, \
    IDCSource, IACSource, INCSource, ILeakyIntegratorExp, ISpikeInjector, \
    IFixedSpikeGenerator, ISpikeRecorder, IPoissonSpikeGenerator, IPopulationRate

from hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter import PyNNCommunicationAdapter
from .devices import PyNNSpiNNakerACSource, PyNNSpiNNakerDCSource, \
    PyNNSpiNNakerLeakyIntegratorAlpha, PyNNSpiNNakerNCSource, PyNNSpiNNakerFixedSpikeGenerator, \
    PyNNSpiNNakerSpikeRecorder, PyNNSpiNNakerPoissonSpikeGenerator, \
    PyNNSpiNNakerLeakyIntegratorExp, PyNNSpiNNakerPopulationRate, SpiNNakerSpikeInjector
from hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection import \
    reset as reset_connection, \
    shutdown
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connections

logger = logging.getLogger(__name__)

__author__ = "Georg Hinkel"


class PyNNSpiNNakerCommunicationAdapter(PyNNCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IFixedSpikeGenerator: PyNNSpiNNakerFixedSpikeGenerator,
                     IDCSource: PyNNSpiNNakerDCSource,
                     IACSource: PyNNSpiNNakerACSource,
                     INCSource: PyNNSpiNNakerNCSource,
                     ILeakyIntegratorAlpha: PyNNSpiNNakerLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: PyNNSpiNNakerLeakyIntegratorExp,
                     ISpikeRecorder: PyNNSpiNNakerSpikeRecorder,
                     IPoissonSpikeGenerator: PyNNSpiNNakerPoissonSpikeGenerator,
                     IPopulationRate: PyNNSpiNNakerPopulationRate,
                     ISpikeInjector: SpiNNakerSpikeInjector}

    def initialize(self):
        """
        Marks the PyNN adapter as initialized
        """
        super(PyNNSpiNNakerCommunicationAdapter, self).initialize()
        live_connections.default_sender = None
        live_connections.default_receiver = None
        live_connections.default_poisson = None
        reset_connection()

    def shutdown(self):
        """
        Shuts down the brain communication adapter
        """
        super(PyNNSpiNNakerCommunicationAdapter, self).shutdown()
        live_connections.shutdown()
        shutdown()

    def _get_device_type(self, device_type):
        """
        Returns the pynn specific implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete type
        """

        if device_type in self.__device_dict:
            return self.__device_dict[device_type]
        return super(PyNNSpiNNakerCommunicationAdapter, self)._get_device_type(device_type)
