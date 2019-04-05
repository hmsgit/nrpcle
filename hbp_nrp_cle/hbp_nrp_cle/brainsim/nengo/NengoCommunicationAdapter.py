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
The brain communication adapter for the Nengo default simulator
"""

import logging
from hbp_nrp_cle.brainsim.BrainInterface import IRawSignal, ISpikeRecorder

from hbp_nrp_cle.brainsim.common import AbstractCommunicationAdapter, DeviceCommunicationDirection
from hbp_nrp_cle.brainsim.nengo.devices import NengoRawSignalNode, NengoSpikeRecorder
from hbp_nrp_cle.brainsim.nengo import NengoInfo

logger = logging.getLogger(__name__)

__author__ = 'Sebastian Krach'


class NengoCommunicationAdapter(AbstractCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """

    __device_dict = {IRawSignal: NengoRawSignalNode,
                     ISpikeRecorder: NengoSpikeRecorder}

    def __init__(self, nengo_simulation_state):
        super(NengoCommunicationAdapter, self).__init__()
        self._nengo_simulation_state = nengo_simulation_state

    def initialize(self):
        """
        Marks the Nengo adapter as initialized
        """
        logger.info("Nengo communication adapter initialized")
        super(NengoCommunicationAdapter, self).initialize()

    def register_spike_source(self, populations, spike_generator_type, **params):
        create_params = params.copy()
        create_params.update(direction_kind=DeviceCommunicationDirection.IN,
                             nengo_simulation_state=self._nengo_simulation_state)
        device = super(NengoCommunicationAdapter, self).register_spike_source(populations,
                                                                              spike_generator_type,
                                                                              **create_params)
        return device

    def register_spike_sink(self, populations, spike_detector_type, **params):
        create_params = params.copy()
        create_params.update(direction_kind=DeviceCommunicationDirection.OUT,
                             nengo_simulation_state=self._nengo_simulation_state)
        device = super(NengoCommunicationAdapter, self).register_spike_sink(populations,
                                                                            spike_detector_type,
                                                                            **create_params)
        return device

    def _get_device_type(self, device_type):
        """
        Returns the specific implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete type
        """
        if device_type in self.__device_dict:
            return self.__device_dict[device_type]
        return super(NengoCommunicationAdapter, self)._get_device_type(device_type)

    def is_population(self, population):  # -> Boolean:
        """
        Determines whether the given object is a population

        :param population: The object that may be a population
        """
        return NengoInfo.is_population(population)

    def create_view(self, population, sl):  # -> Object:
        """
        Creates a view of the given population

        :param population: The base population
        :param sl: The slice of the population that represents the view
        """
        return NengoInfo.create_view(population, sl)
