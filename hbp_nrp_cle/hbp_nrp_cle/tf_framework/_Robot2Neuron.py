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
This module contains the representation of transfer functions from the world simulator towards the
neuronal simulator
"""

__author__ = 'Georg Hinkel'

from hbp_nrp_cle.robotsim.RobotInterface import Topic, IRobotCommunicationAdapter
from . import config
from ._TransferFunction import TransferFunction
from ._MappingSpecification import ParameterMappingSpecification

import logging
logger = logging.getLogger(__name__)


class MapRobotPublisher(ParameterMappingSpecification):
    """
    Class to map parameters to robot publishers
    """

    def __init__(self, key, value, **kwargs):  # -> None:
        """
        Maps a parameter to a robot topic

        :param key: the name of the parameter
        :param value: the value for the parameter
        :param kwargs: Additional configuration parameters
        """
        super(MapRobotPublisher, self).__init__(key)
        assert isinstance(value, (Topic, str))
        self.__value = value
        self.__config = kwargs

    @property
    def topic(self):
        """
        Gets the topic this mapping directs to
        """
        return self.__value

    @property
    def config(self):
        """
        Gets additional configuration for this mapping
        """
        return self.__config

    def is_robot_connection(self):
        """
        Returns whether the parameter is connected to the simulated robot

        :return: True, if the parameter is mapped to the simulated robot, otherwise False
        """
        return True

    def create_adapter(self, transfer_function_manager):
        """
        Creates the adapter for this mapping operator
        """
        adapter = transfer_function_manager.robot_adapter
        assert isinstance(adapter, IRobotCommunicationAdapter)
        return adapter.register_publish_topic(self.topic, **self.config)

    def create_tf(self):
        """
        Creates a TF in case the TF specification has been omitted
        """
        from hbp_nrp_cle.tf_framework._Neuron2Robot import Neuron2Robot
        return Neuron2Robot()


class MapRobotSubscriber(MapRobotPublisher):
    """
    Represents a parameter mapping to a robot subscriber
    """
    def create_adapter(self, transfer_function_manager):
        """
        Creates the adapter for this mapping operator
        """
        adapter = transfer_function_manager.robot_adapter
        assert isinstance(adapter, IRobotCommunicationAdapter)
        return adapter.register_subscribe_topic(self.topic, **self.config)

    def create_tf(self):
        """
        Creates a TF in case the TF specification has been omitted
        """
        return Robot2Neuron()


class Robot2Neuron(TransferFunction):
    """
    Represents a transfer function from robot topics to neurons
    """

    def __call__(self, func):  # -> Robot2Neuron:
        """
        Attaches the given function to the current transfer function object

        :param func: The function implementing the transfer function
        :return: The transfer function object
        """
        self._init_function(func, config.active_node.r2n)
        return self

    def __repr__(self):  # pragma: no cover
        return "{0} transfers to neurons {1}".format(self.name, self._params)

    def unregister(self):
        pass
