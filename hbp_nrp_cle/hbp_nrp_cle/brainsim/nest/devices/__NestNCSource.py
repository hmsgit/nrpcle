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
Implementation of NestCSource
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import INCSource

import nest

__author__ = 'LorenzoVannucci'


class NestNCSource(AbstractBrainDevice, INCSource):
    """
    Represents a noisy current generator
    :param kwargs: Optional configuration parameters
    """

    default_parameters = {
        'mean': 0.0,
        'std': 1000.0,
        "start": 0.0,
        "stop": float("inf")
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a noisy current generator.

        :param mean: Mean value of the noisy current, default: 0.0 pA
        :param stdev: Standard deviation of the noisy current, default: 1000.0 pA
        :param dt: Interval between updates of the current amplitude in ms,
            default: simulation time step
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """
        super(NestNCSource, self).__init__(**params)

        self._generator = None
        self._last_params_before_deactivation = {
            'mean': None,
            'stdev': None,
        }
        self.create_device()

    @property
    def mean(self):
        """
        Returns the mean of the current
        """
        return nest.GetStatus(self._generator, 'mean')[0]

    @mean.setter
    def mean(self, value):  # pragma: no cover
        """
        Sets the mean of the current

        :param value: float
        """
        nest.SetStatus(self._generator, {'mean': value})

    @property
    def stdev(self):
        """
        Returns the stdev of the current
        """
        return nest.GetStatus(self._generator, 'std')[0]

    @stdev.setter
    def stdev(self, value):  # pragma: no cover
        """
        Sets the stdev of the current

        :param value: float
        """
        nest.SetStatus(self._generator, {'std': value})

    def _activate(self):
        """Activate this source, if inactive, restoring the previous stdev and mean values"""
        if not self.active:
            self.stdev = self._last_params_before_deactivation['stdev']
            self.mean = self._last_params_before_deactivation['mean']

    def _deactivate(self):
        """Activate this source, if inactive, setting  stdev and mean to zero"""
        if self.active:
            self._last_params_before_deactivation['stdev'] = self.stdev
            self._last_params_before_deactivation['mean'] = self.mean
            self.stdev = 0.0
            self.mean = 0.0

    def create_device(self):
        """
        Create a noisy current source

        """
        self._generator = nest.Create('noise_generator', 1,
                                      self.get_parameters("mean",
                                                          "std",
                                                          "start",
                                                          "stop"))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        nest.Connect(self._generator, neurons)

    def _disconnect(self):
        """
        Disconnects the device by setting mean and stdev to 0 since we cannot delete the device or
        connection directly via NEST.
        """
        if self._generator:
            self.mean = 0.0
            self.stdev = 0.0
            self._generator = None
