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

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IACSource

__author__ = 'DimitriProbst, Sebastian Krach'


class PyNNACSource(AbstractBrainDevice, IACSource):
    """
    Represents an alternating current generator.
    """

    default_parameters = {
        "amplitude": 0.0,
        "offset": 0.0,
        "frequency": 10.0,
        "phase": 0.0,
        "start": 0.0,
        "stop": float("inf")
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        :param offset: Offset of alternating current, default: 0.0 nA
        :param frequency: Frequency of alternating current, default: 10 Hz
        :param phase: Phase of alternating current, default: 0.0
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """

        super(PyNNACSource, self).__init__(**params)

        self._generator = None

        self._last_amplitude_before_deactivation = None

        self.create_device()

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self._generator.amplitude

    @amplitude.setter
    def amplitude(self, value):  # pragma: no cover
        """
        Sets the amplitude of the current

        :param value: float
        """
        self._generator.set(amplitude=value)

    def _activate(self):
        """Activate this source, if inactive,
        restoring the previous amplitude"""
        if not self.active:
            self.amplitude = self._last_amplitude_before_deactivation

    def _deactivate(self):
        """Deactivate this source, if inactive,
        setting amplitude to zero"""
        if self.active:
            self._last_amplitude_before_deactivation = self.amplitude

            self.amplitude = 0.0

    @property
    def offset(self):
        """
        Returns the offset of the current
        """
        return self._generator.offset

    @offset.setter
    def offset(self, value):  # pragma: no cover
        """
        Sets the offset of the current

        :param value: float
        """
        self._generator.set(offset=value)

    @property
    def frequency(self):
        """
        Returns the frequency of the current
        """
        return self._generator.frequency

    @frequency.setter
    def frequency(self, value):  # pragma: no cover
        """
        Sets the frequency of the current

        :param value: float
        """
        self._generator.set(frequency=value)

    @property
    def phase(self):
        """
        Returns the phase of the current
        """
        return self._generator.phase

    @phase.setter
    def phase(self, value):  # pragma: no cover
        """
        Sets the phase of the current

        :param value: float
        """
        self._generator.set(phase=value)

    def sim(self):  # pragma: no cover
        """
        Gets the simulator module to use
        """
        raise NotImplementedError("This method must be overridden in a derived class")

    def create_device(self):
        """
        Creates an alternating current source
        """
        self._generator = self.sim().ACSource(**self.get_parameters("amplitude",
                                                                    "offset",
                                                                    "frequency",
                                                                    "phase",
                                                                    "start",
                                                                    "stop"))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._generator.inject_into(neurons)

    def _disconnect(self):
        """
        Disconnects the device by setting all output to 0 since we cannot delete the device or
        connection directly via PyNN.
        """
        if self._generator:
            self.amplitude = 0.0
            self.offset = 0.0
            self.frequency = 0.0
            self.phase = 0.0
            self._generator = None
