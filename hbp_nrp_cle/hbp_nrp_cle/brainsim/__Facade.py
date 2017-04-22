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
This module provides interface functionality abstracting from different implementations of the
brain adapter. Concrete implementations should not be instantiated directly but through the
provided functionality in this module.

The purpose of the abstraction is to facilitate later exchanges of the brain adapter implementation.
"""

__author__ = "Sebastian Krach"

from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter
from hbp_nrp_cle.brainsim.pynn_nest.PyNNNestCommunicationAdapter import PyNNNestCommunicationAdapter
from . import config

DEFAULT_COMMUNICATION_ADAPTER = PyNNNestCommunicationAdapter
DEFAULT_CONTROL_ADAPTER = PyNNControlAdapter


def initialize_defaults():
    """
    Sets the brain simulator config to the default PyNN implementation
    """

    config.communication_adapter_type = DEFAULT_COMMUNICATION_ADAPTER
    config.control_adapter_type = DEFAULT_CONTROL_ADAPTER


def instantiate_communication_adapter():
    """
    Instantiates a new IBrainCommunicationAdapter according to the currently configured brain
    simulator adapter implementation.

    :return: an IBrainCommunicationAdapter instance
    """
    if not config.communication_adapter_type:
        initialize_defaults()
    return config.communication_adapter_type()


def instantiate_control_adapter():
    """
    Instantiates a new IBrainControlAdapter according to the currently configured brain
    simulator adapter implementation.

    :return: an IBrainControlAdapter instance
    """
    if not config.control_adapter_type:
        initialize_defaults()
    return config.control_adapter_type()
