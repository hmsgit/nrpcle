"""
This module contains the representation from transfer functions from the neuronal simulator towards
the world simulator
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import Topic
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, \
    ISpikeDetector, IDCSource, IACSource, INCSource, IPopulationRate, \
    ICustomDevice, IBrainCommunicationAdapter
from . import config
from ._TransferFunction import TransferFunction

import inspect


class MapSpikeSink(object):
    """
    Class to map parameters to spike sinks such as leaky integrators
    """

    supported_device_types = [ISpikeDetector, ILeakyIntegratorAlpha, ILeakyIntegratorExp,
                              IPopulationRate]

    def __init__(self, key, value, device_type, **kwargs):  # -> None:
        """
        Maps a parameter to a neuron
        :param key: the parameter name
        :param value: the neuron reference
        :param kwargs: Additional configuration
        :param device_type: The type of device that should be created at the referenced neurons
        """
        self.__value = value
        self.__key = key
        if not isinstance(device_type, ICustomDevice):
            if not self.is_supported(device_type):
                raise Exception("Device type is not supported")
        self.__device_type = device_type
        self.__config = kwargs

    def __call__(self, transfer_function):  # -> object:
        """
        Applies the parameter mapping to the given transfer function
        """
        if isinstance(transfer_function, TransferFunction):
            neurons = transfer_function.params
            for i in range(0, len(neurons)):
                if neurons[i] == self.__key:
                    neurons[i] = self
                    return transfer_function
        else:
            raise Exception("Can only map parameters for transfer functions")
        raise Exception(
            "Could not map parameter as no parameter with the name " + self.__key + " exists")

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
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__key

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        adapter = transfer_function_manager.brain_adapter
        assert isinstance(adapter, IBrainCommunicationAdapter)
        return adapter.register_spike_sink(config.brain_root.actors,
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
        return adapter.register_spike_source(config.brain_root.sensors,
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
        self.__func = None

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
        self.__func = func
        n2r_funcs = config.active_node.n2r
        n2r_funcs.append(self)
        args = inspect.getargspec(func).args
        if args[0] != "t":
            raise Exception("The first parameter of a transfer function must be the time!")
        self._params = list(args)
        return self

    def replace_params(self):  # -> None:
        """
        Replaces strings to neuron references
        if the parameters are not mapped to neurons, voltmeters are generated
        """
        for i in range(1, len(self._params)):
            if type(self._params[i]) == str:
                param_name = self._params[i].lower()
                gid = None
                if param_name.startswith("neuron"):
                    gid = int(param_name[6:])
                elif param_name.startswith("n"):
                    gid = int(param_name[1:])
                self._params[i] = MapSpikeSink(self._params[i], [gid],
                                                     ILeakyIntegratorAlpha)

    def __repr__(self):  # pragma: no cover
        return "{0} transfers to robot {1} using {2}" \
            .format(self.__func, self.__main_topic, self._params)

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time
        :param t: The simulation time
        """
        self._params[0] = t
        return_value = self.__func(*self._params)
        if return_value is not None:
            topic_publisher = self.__main_topic
            if topic_publisher is not None:
                topic_publisher.send_message(return_value)
