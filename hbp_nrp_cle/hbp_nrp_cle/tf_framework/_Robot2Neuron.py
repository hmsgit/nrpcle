"""
This module contains the representation of transfer functions from the world simulator towards the
neuronal simulator
"""

__author__ = 'GeorgHinkel'

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
        :param subscribe: A boolean value indicating whether the topic should be subscribed
         or published to
        :param kwargs: Additional configuration parameters
        """
        super(MapRobotPublisher, self).__init__(key)
        assert isinstance(value, Topic)
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
