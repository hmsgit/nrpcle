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
The device realizes the raw signal data exchange functionality for the Nengo default simulator
"""

import logging

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IRawSignal
from hbp_nrp_cle.brainsim.common import DeviceCommunicationDirection
import nengo
import numpy

logger = logging.getLogger("NengoRawSignalNode")


class NengoRawSignalNode(AbstractBrainDevice, IRawSignal):
    """
    Determines the raw signal data exchange for the Nengo simulator
    """

    def __init__(self, direction_kind, nengo_simulation_state, **kwargs):
        super(NengoRawSignalNode, self).__init__(**kwargs)
        self._direction_kind = direction_kind
        self.__value = None
        self._internal_device = None
        self._internal_device_connection = None
        self._nengo_simulation_state = nengo_simulation_state

    @property
    def value(self):
        return self.__value

    @value.setter
    #pylint: disable=arguments-differ
    def value(self, new_value):
        self.__value = new_value

    def _init_internal_value(self, size_neurons):
        """
        Initializes the internal value storage based on the neuron population size with a
        sensible default value (0.0 by default)
        :param size_neurons: the number of neurons, and therefore the number of result dimensions
        """
        if size_neurons > 1:
            self.__value = numpy.zeros(size_neurons, dtype=float)
        elif size_neurons == 1:
            self.__value = 0.0
        else:
            raise Exception("No neurons provided to connect against.")

    def _select_presynaptic_entity(self, neurons, internal_device):
        """
        Select from the two parameters the one which is the presynaptic entity of the
        Connection to be created
        :param neurons: The Nengo ensemble to connect against
        :param internal_device: The Nengo entity created by this device
        :return: the presynaptic entity
        """
        return internal_device \
            if self._direction_kind == DeviceCommunicationDirection.IN else neurons

    def _select_postsynaptic_entity(self, neurons, internal_device):
        """
        Select from the two parameters the one which is the postsynaptic entity of the
        Connection to be created
        :param neurons: The Nengo ensemble to connect against
        :param internal_device: The Nengo entity created by this device
        :return: the postsynaptic entity
        """
        return neurons \
            if self._direction_kind == DeviceCommunicationDirection.IN else internal_device

    def _get_node_parameters(self, neurons):
        """
        Returns the appropriate parameters to pass to the Nengo Node object.
        :param neurons: The ensemble to connect against
        :return: the appropriate parameters
        """
        if self._direction_kind == DeviceCommunicationDirection.IN:
            return {"size_in": 0,
                    "size_out": len(neurons)}
        else:
            return {"size_in": len(neurons),
                    "size_out": 0}

    # pylint: disable=unused-argument
    def _get_connection_parameters(self, neurons):
        """
        Returns the appropriate parameters to pass to the Nengo Connection object.
        :param neurons: The ensemble to connect against
        :return: the appropriate parameters
        """
        if self._direction_kind == DeviceCommunicationDirection.IN:
            return {}
        else:
            return {"synapse": 0.02}

    def connect(self, neurons):
        with self._nengo_simulation_state:
            size_neurons = len(neurons)

            self._init_internal_value(size_neurons)

            if self._direction_kind == DeviceCommunicationDirection.IN:
                # pylint: disable=unused-argument
                def node_function(t):
                    """
                    The getter function which is executed by the Nengo simulator
                    """
                    return self.__value
            else:
                # pylint: disable=unused-argument
                def node_function(t, x):
                    """
                    The setter function which is executed by the Nengo simulator
                    """
                    self.__value = x

            self._internal_device = nengo.Node(node_function, **self._get_node_parameters(neurons))
            self._internal_device_connection = nengo.Connection(
                self._select_presynaptic_entity(neurons, self._internal_device),
                self._select_postsynaptic_entity(neurons, self._internal_device),
                ** self._get_connection_parameters(neurons))

    def _disconnect(self):
        self._nengo_simulation_state.delete_from_brain(self._internal_device)
        self._nengo_simulation_state.delete_from_brain(self._internal_device_connection)
