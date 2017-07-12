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
This module contains the MockBrainCommunicationAdapter which can be used for running the cle
without a brain simulator backend.
'''

from hbp_nrp_cle.brainsim.common import AbstractCommunicationAdapter
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, ICustomDevice, \
    IFixedSpikeGenerator, ILeakyIntegratorExp, IPopulationRate, ISpikeRecorder
from .__devices.MockCurrentSource import MockACSource, MockDCSource
from .__devices.MockSpikeGenerator import MockFixedSpikeGenerator, MockPoissonSpikeGenerator
from .__devices.MockNCSource import MockNCSource
from .__devices.MockLeakyIntegrator import MockLeakyIntegratorAlpha, MockLeakyIntegratorExp
from .__devices.MockPopulationRate import MockPopulationRate
from .__devices.MockSpikeRecorder import MockSpikeRecorder

__author__ = 'MichaelWeber'


class MockBrainCommunicationAdapter(AbstractCommunicationAdapter):
    """
    The MockBrainCommunicationAdapter implements the IBrainCommunicationAdapter interface with
    mock capabilities. It can be used to run the CLE without having a brain simulation attached.
    """

    __device_dict = {IFixedSpikeGenerator: MockFixedSpikeGenerator,
                     IPoissonSpikeGenerator: MockPoissonSpikeGenerator,
                     IDCSource: MockDCSource,
                     IACSource: MockACSource,
                     INCSource: MockNCSource,
                     ILeakyIntegratorAlpha: MockLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: MockLeakyIntegratorExp,
                     IPopulationRate: MockPopulationRate,
                     ISpikeRecorder: MockSpikeRecorder}

    def __init__(self):
        """
        Initializes a new mocked brain communication adapter
        """
        super(MockBrainCommunicationAdapter, self).__init__()

        self.__refreshed_at = []

    def initialize(self):
        """
        Marks the mock adapter as initialized
        """
        super(MockBrainCommunicationAdapter, self).initialize()

    def _get_device_type(self, device_type):
        """
        Returns the specific mock implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete mock type
        """
        return self.__device_dict[device_type]

    def register_spike_source(self, neurons, spike_generator_type, **params):
        """
        Requests a communication object with the given spike generator type
        for the given set of neurons

        :param neurons: A reference to the neurons to which the spike generator
         should be connected
        :param spike_generator_type: A spike generator type (see documentation
         or a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A communication object
        """
        if isinstance(spike_generator_type, ICustomDevice):
            spike_generator_type.apply(neurons, self, **params)
            return spike_generator_type

        return super(MockBrainCommunicationAdapter, self).register_spike_source(
            neurons, spike_generator_type, **params)

    def register_spike_sink(self, neurons, spike_detector_type, **params):
        """
        Requests a communication object with the given spike detector type
        for the given set of neurons

        :param neurons: A reference to the neurons which should be connected
         to the spike detector
        :param spike_detector_type: A spike detector type (see documentation
         for a list of allowed values)
        :param params: A dictionary of configuration parameters
        :return: A Communication object
        """
        if isinstance(spike_detector_type, ICustomDevice):
            spike_detector_type.apply(neurons, self, **params)
            return spike_detector_type

        return super(MockBrainCommunicationAdapter, self).register_spike_sink(
            neurons, spike_detector_type, **params)

    def refresh_buffers(self, t):
        """
        Refreshes buffered values for time t

        :param t: The brain simulation time
        """
        self.__refreshed_at.append(t)
        super(MockBrainCommunicationAdapter, self).refresh_buffers(t)

    @property
    def refreshed_at(self):
        """
        Gets a list of simulation times at which the mock has been refreshed
        """
        return self.__refreshed_at

    def shutdown(self):
        pass

    def is_population(self, population):  # pragma: no cover
        """
        Determines whether the given object is a population

        :param population: The object that may be a population
        """
        return True

    def create_view(self, population, sl):  # -pragma: no cover
        """
        Creates a view of the given population

        :param population: The base population
        :param sl: The slice of the population that represents the view
        """
        return population[sl]
