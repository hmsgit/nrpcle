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
"""
This module contains a test device that is used to test the functionality of custom devices
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.brainsim.BrainInterface import ICustomDevice, ILeakyIntegratorAlpha, IACSource, \
    IBrainCommunicationAdapter


class TestDevice(ICustomDevice):
    """
    Represents a device for testing purposes
    """

    def __init__(self):
        """
        Creates a new test device
        """
        self.__source = None

    def apply(self, neurons, brain_adapter, **config):
        """
        Binds the test device to the specified set of neurons
        :param neurons: The neurons
        :param brain_adapter: The brain adapter
        :param config: Additional device configuration
        """
        assert isinstance(brain_adapter, IBrainCommunicationAdapter)
        self.__source = brain_adapter.register_spike_source(neurons, IACSource, **config)

    @property
    def inner(self):
        """
        Gets the inner AC source managed by this test device
        """
        return self.__source

    def reset(self, transfer_function_manager):
        """
        Resets the test device
        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
        """
        new_device = TestDevice()
        new_device.__source = self.__source.reset(transfer_function_manager)
        return new_device