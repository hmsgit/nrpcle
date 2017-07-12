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
This module contains the shared funtionality among multiple mock brain device implementations.
"""

__author__ = "Sebastian Krach"

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice, DeviceGroup


class MockDeviceGroup(DeviceGroup):
    """
    The class adds functionality to the DeviceGroup implementation to provide future values for
    the nested mock devices
    """

    @classmethod
    def _create_device_config(cls, params, index):
        """
        Creates the configuration for the device with the given index

        :param params: The original parameters
        :return: A parameter array
        """
        new_params = {}
        for key in params:
            value = params[key]
            if key == 'updates':
                new_value = []
                for (t, values) in value:
                    new_value.append((t, values[index]))
                value = new_value
            else:
                if hasattr(value, '__getitem__'):
                    value = value[index]
            new_params[key] = value
        print "mock create device config: %s" % new_params
        return new_params


class AbstractMockBrainDevice(AbstractBrainDevice):
    """
    Shared abstract super class for all mock devices which implements the unused methods to
    reduce the mock implementation file size.
    """

    def reset(self, transfer_function_manager):
        pass

    def connect(self, neurons, **params):
        pass

    def _disconnect(self):
        pass

    @classmethod
    def create_new_device_group(cls, length, params):
        return MockDeviceGroup.create_new_device_group(cls, length, params)
