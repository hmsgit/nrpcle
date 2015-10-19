"""
This module contains the representation of transfer functions from the world simulator towards the
neuronal simulator
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import Topic, IRobotCommunicationAdapter
from . import config
from ._TransferFunction import TransferFunction

import logging
logger = logging.getLogger(__name__)


class MapRobotPublisher(object):
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
        assert isinstance(value, Topic)
        self.__key = key
        self.__value = value
        self.__config = kwargs

    def __call__(self, transfer_function):  # -> Robot2Neuron:
        """
        Applies the parameter mapping to the given transfer function
        """
        assert isinstance(transfer_function, TransferFunction)
        topics = transfer_function.params
        for i in range(0, len(topics)):
            if topics[i] == self.__key:
                topics[i] = self
                return transfer_function
        raise Exception("Could not map parameter as no parameter with the given name exists")

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

    @property
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__key

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
