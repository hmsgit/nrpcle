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
NestCommunicationAdapter
"""

import logging
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, ILeakyIntegratorExp, IPopulationRate, \
    IFixedSpikeGenerator, ISpikeRecorder

from hbp_nrp_cle.brainsim.common import AbstractCommunicationAdapter
from hbp_nrp_cle.brainsim.nest.NestInfo import is_population, create_view
from hbp_nrp_cle.brainsim.nest.devices import NestPopulationRate, NestACSource, NestDCSource, \
    NestFixedSpikeGenerator, NestLeakyIntegratorAlpha, NestLeakyIntegratorExp, NestNCSource, \
    NestPoissonSpikeGenerator, NestSpikeRecorder


logger = logging.getLogger(__name__)


class NestCommunicationAdapter(AbstractCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IFixedSpikeGenerator: NestFixedSpikeGenerator,
                     IPoissonSpikeGenerator: NestPoissonSpikeGenerator,
                     IDCSource: NestDCSource,
                     IACSource: NestACSource,
                     INCSource: NestNCSource,
                     ILeakyIntegratorAlpha: NestLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: NestLeakyIntegratorExp,
                     IPopulationRate: NestPopulationRate,
                     ISpikeRecorder: NestSpikeRecorder
                     }

    def initialize(self):
        """
        Marks the NEST adapter as initialized
        """
        logger.info("NEST communication adapter initialized")
        super(NestCommunicationAdapter, self).initialize()

    def _get_device_type(self, device_type):
        """
        Returns the NEST specific implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete type
        """
        if device_type in self.__device_dict:
            return self.__device_dict[device_type]
        return super(NestCommunicationAdapter, self)._get_device_type(device_type)

    def is_population(self, population):  # -> Boolean:
        """
        Determines whether the given object is a population

        :param population: The object that may be a population
        """
        return is_population(population)

    def create_view(self, population, sl):  # -> Object:
        """
        Creates a view of the given population

        :param population: The base population
        :param sl: The slice of the population that represents the view
        """
        return create_view(population, sl)
