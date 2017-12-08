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
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNACSource
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice, \
    create_transformation
import pyNN.nest as nestsim

__author__ = 'Georg Hinkel, Dimitri Probst'


class PyNNNestACSource(PyNNNestDevice, PyNNACSource):
    """
    Represents an alternating current generator.
    """

    transformations = {
        "amplitude": create_transformation("amplitude", 1000.0),
        "offset": create_transformation("offset", 1000.0),
        "frequency": create_transformation("frequency"),
        "phase": create_transformation("phase")
    }

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self._parameters["amplitude"]

    def sim(self):
        """
        Gets the simulator module to use
        """
        return nestsim

    # Pylint does not really recognize property overrides
    # pylint: disable=arguments-differ
    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        if value != self.amplitude:
            self._parameters["amplitude"] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {'amplitude': 1000.0 * value})

    @property
    def offset(self):
        """
        Returns the offset of the current
        """
        return self._parameters['offset']

    # Pylint does not really recognize property overrides
    # pylint: disable=arguments-differ
    @offset.setter
    def offset(self, value):
        """
        Sets the offset of the current

        :param value: float
        """
        if self.offset != value:
            self._parameters["offset"] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {'offset': 1000.0 * value})

    @property
    def frequency(self):
        """
        Returns the frequency of the current
        """
        return self._parameters['frequency']

    # Pylint does not really recognize property overrides
    # pylint: disable=arguments-differ
    @frequency.setter
    def frequency(self, value):
        """
        Sets the frequency of the current

        :param value: float
        """
        if self.frequency != value:
            self._parameters['frequency'] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {'frequency': float(value)})

    @property
    def phase(self):
        """
        Returns the phase of the current
        """
        return self._parameters['phase']

    # Pylint does not really recognize property overrides
    # pylint: disable=arguments-differ
    @phase.setter
    def phase(self, value):
        """
        Sets the phase of the current

        :param value: float
        """
        if self.phase != value:
            self._parameters['phase'] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {'phase': float(value)})
