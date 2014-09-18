__author__ = 'GeorgHinkel'

from python_cle.robotsim.RobotInterface import Topic
from python_cle.brainsim.BrainInterface import INeuronVoltmeter, ISpikeRecorder, \
    IPoissonSpikeGenerator, IFixedFrequencySpikeGenerator, IPatternSpikeGenerator, ICustomDevice
from .Robot2Neuron import Robot2Neuron
from . import config

import inspect


class MapNeuronParameter(object):
    """
    Class to map parameters to neurons
    """

    supported_device_types = [ISpikeRecorder, INeuronVoltmeter, IPoissonSpikeGenerator,
                              IFixedFrequencySpikeGenerator, IPatternSpikeGenerator]

    def __init__(self, key, value, device_type, **kwargs):  # -> None:
        """
        Maps a parameter to a neuron
        :param key: the parameter name
        :param value: the neuron reference
        :param kwargs: Additional configuration
        :param device_type: The type of device that should be created at the referenced neurons
        """
        if isinstance(value, int):
            self.__value = [value]
        else:
            if isinstance(value, list):
                self.__value = value
            else:
                self.__value = list(value)
        self.__key = key
        if not isinstance(device_type, ICustomDevice):
            if not device_type in MapNeuronParameter.supported_device_types:
                raise Exception("Device type is not supported")
        self.__device_type = device_type
        self.__config = kwargs

    def __call__(self, n2r):  # -> object:
        """
        Applies the parameter mapping to the given transfer function
        """
        if isinstance(n2r, Neuron2Robot):
            neurons = n2r.neuron_params
            for i in range(0, len(neurons)):
                if neurons[i] == self.__key:
                    neurons[i] = self
                    return n2r
        elif isinstance(n2r, Robot2Neuron):
            params = n2r.params
            for i in range(0, len(params)):
                if params[i] == self.__key:
                    params[i] = self
                    return n2r
        else:
            raise Exception("Can only map parameters for neuron2robot objects")
        raise Exception(
            "Could not map parameter as no parameter with the name " + self.__key + " exists")

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


class Neuron2Robot(object):
    """
    Class to represent a transfer function from neurons to robot
    """

    def __init__(self, robot_topic, *other_topics):  # -> None:
        """
        Defines a new transfer function from robots to neurons
        :param robot_topic: the robot topic reference
        :param other_topics: other topics required by this transfer function
        """
        assert isinstance(robot_topic, Topic)
        self.__main_robot_topic = robot_topic
        self.__robot_topics = other_topics
        self.__func = None
        self.__neuron_params = []

    def __get_neuron_params(self):  # -> list:
        return self.__neuron_params

    def __get_main_robot_topic(self):  # -> Topic:
        return self.__main_robot_topic

    def __set_main_robot_topic(self, robot_topic):  # -> None:
        self.__main_robot_topic = robot_topic

    def __get_robot_topics(self):  # -> list:
        return self.__robot_topics

    # The main robot topic is the topic that the return value of the transfer function is connected
    # to
    main_robot_topic = property(__get_main_robot_topic, __set_main_robot_topic)

    # The neuron parameters are descriptions of the parameters that the transfer functions takes as
    # inputs
    neuron_params = property(__get_neuron_params)

    # The robot topics are the robot topics that may be accessed by the current transfer function
    robot_topics = property(__get_robot_topics)

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
        self.__neuron_params = list(args)
        return self

    def replace_params(self):  # -> None:
        """
        Replaces strings to neuron references
        if the parameters are not mapped to neurons, voltmeters are generated
        """
        for i in range(1, len(self.__neuron_params)):
            if type(self.__neuron_params[i]) == str:
                param_name = self.__neuron_params[i].lower()
                gid = None
                if param_name.startswith("neuron"):
                    gid = int(param_name[6:])
                elif param_name.startswith("n"):
                    gid = int(param_name[1:])
                self.__neuron_params[i] = MapNeuronParameter(None, [gid], INeuronVoltmeter)

    def __repr__(self):  # pragma: no cover
        return "{0} transfers to robot {1} {2} using {3}" \
            .format(self.__func, self.__main_robot_topic, self.__robot_topics, self.__neuron_params)

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time
        :param t: The simulation time
        """
        self.__neuron_params[0] = t
        return_value = self.__func(*self.__neuron_params)
        if return_value is not None:
            self.__main_robot_topic.send_message(return_value)
