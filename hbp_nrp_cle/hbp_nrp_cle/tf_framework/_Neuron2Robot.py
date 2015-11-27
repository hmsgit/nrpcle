"""
This module contains the representation from transfer functions from the neuronal simulator towards
the world simulator
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import Topic
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, IPopulationRate, \
    ICustomDevice, IBrainCommunicationAdapter, ISpikeRecorder
from ._MappingSpecification import ParameterMappingSpecification
from . import config
from ._TransferFunction import TransferFunction

import logging
logger = logging.getLogger(__name__)


class MapSpikeSink(ParameterMappingSpecification):
    """
    Class to map parameters to spike sinks such as leaky integrators
    """

    supported_device_types = [ILeakyIntegratorAlpha, ILeakyIntegratorExp,
                              IPopulationRate, ISpikeRecorder]

    def __init__(self, key, value, device_type, **kwargs):  # -> None:
        """
        Maps a parameter to a neuron

        :param key: the parameter name
        :param value: the neuron reference
        :param kwargs: Additional configuration
        :param device_type: The type of device that should be created at the referenced neurons
        """
        super(MapSpikeSink, self).__init__(key)
        self.__value = value
        if not isinstance(device_type, ICustomDevice):
            if not self.is_supported(device_type):
                raise Exception("Device type is not supported")
        self.__device_type = device_type
        self.__config = kwargs

    def is_supported(self, device_type):  # pylint: disable=R0201
        """
        Gets a value indicating whether the given device type is supported

        :param device_type: The device type that should be looked for
        """
        return device_type in MapSpikeSink.supported_device_types

    @property
    def neurons(self):
        """
        Gets the neurons referenced by this mapping
        """
        return self.__value

    @property
    def device_type(self):
        """
        Gets the device type referenced by this mapping
        """
        return self.__device_type

    @property
    def config(self):
        """
        Gets the configuration used by this mapping
        """
        return self.__config

    @property
    def is_brain_connection(self):
        """
        Returns whether the the parameter is connected to the neuronal network

        :return: True, if the parameter is mapped to the neuronal network, otherwise False
        """
        return True

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        adapter = transfer_function_manager.brain_adapter
        assert isinstance(adapter, IBrainCommunicationAdapter)
        neurons = self.neurons.select(config.brain_root)
        return adapter.register_spike_sink(neurons,
                                           self.device_type,
                                           **self.config)


class MapSpikeSource(MapSpikeSink):
    """
    Class to map parameters to spike sources such as poisson generators
    """

    supported_device_types = [IPoissonSpikeGenerator, IFixedSpikeGenerator,
                              IDCSource, IACSource, INCSource]

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        adapter = transfer_function_manager.brain_adapter
        assert isinstance(adapter, IBrainCommunicationAdapter)
        neurons = self.neurons.select(config.brain_root)
        return adapter.register_spike_source(neurons,
                                             self.device_type,
                                             **self.config)

    def is_supported(self, device_type):
        """
        Gets a value indicating whether the given device type is supported

        :param device_type: The device type that should be looked for
        """
        return device_type in MapSpikeSource.supported_device_types


class Neuron2Robot(TransferFunction):
    """
    Class to represent a transfer function from neurons to robot
    """

    def __init__(self, robot_topic=None):
        """
        Defines a new transfer function from robots to neurons

        :param robot_topic: the robot topic reference
        :param other_topics: other topics required by this transfer function
        """
        super(Neuron2Robot, self).__init__()
        if robot_topic is not None:
            assert isinstance(robot_topic, Topic)
        self.__main_topic = robot_topic

    @property
    def topic(self):  # -> Topic:
        """
        The main robot topic is the topic that the return value of the transfer function is
        connected to
        """
        return self.__main_topic

    @topic.setter
    def topic(self, robot_topic):  # -> None:
        """
        Sets the main robot topic

        :param robot_topic: The new main robot topic
        """
        self.__main_topic = robot_topic

    def __call__(self, func):  # -> Neuron2Robot:
        """
        Applies the transfer functions object to the given function

        :param func: The function body for this transfer function
        :return The transfer function object
        """

        self._init_function(func, config.active_node.n2r)
        return self

    def __repr__(self):  # pragma: no cover
        return "{0} transfers to robot {1} using {2}" \
            .format(self.name, self.__main_topic, self._params)

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time

        :param t: The simulation time
        """

        return_value = super(Neuron2Robot, self).run(t)

        if return_value is not None:
            topic_publisher = self.__main_topic
            if topic_publisher is not None:
                topic_publisher.send_message(return_value)
