__author__ = 'GeorgHinkel'

from robotsim.RobotInterface import Topic
from brainsim.BrainInterface import INeuronVoltmeter
from .Robot2Neuron import Robot2Neuron
from . import config

import inspect


class MapNeuronParameter(object):
    """
    Class to map parameters to neurons
    """

    def __init__(self, key, value, device_type):
        """
        Maps a parameter to a neuron
        :param key: the parameter name
        :param value: the neuron reference
        :param device_type: The type of device that should be created at the referenced neurons
        """
        if isinstance(value, int):
            self.__value = [value]
        else:
            if isinstance(value, list):
                self.__value = value
            else:
                self.__value = list(value)
        # TODO: handle the case when the device_type is not a type
        # assert isinstance(device_type, type)
        self.__key = key
        self.__device_type = device_type

    def __call__(self, n2r):
        if isinstance(n2r, Neuron2Robot):
            neurons = n2r.neuron_params
            for i in range(0, len(neurons)):
                if neurons[i] == self.__key:
                    neurons[i] = (self.__value, self.__device_type)
                    return n2r
        elif isinstance(n2r, Robot2Neuron):
            params = n2r.params
            for i in range(0, len(params)):
                if params[i] == self.__key:
                    params[i] = (self.__value, self.__device_type)
                    return n2r
        else:
            raise Exception("Can only map parameters for neuron2robot objects")
        raise Exception("Could not map parameter as no parameter with the name " + self.__key + " exists")


class Neuron2Robot(object):
    """
    Class to represent a transfer function from neurons to robot
    """

    def __init__(self, robot_topic, *other_topics):
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

    def __get_main_robot_topic(self):
        return self.__main_robot_topic

    def __set_main_robot_topic(self, robot_topic):
        self.__main_robot_topic = robot_topic

    def __get_robot_topics(self):
        return self.__robot_topics

    main_robot_topic = property(__get_main_robot_topic, __set_main_robot_topic)

    neuron_params = property(__get_neuron_params)

    robot_topics = property(__get_robot_topics)

    def __call__(self, func):
        self.__func = func
        n2r_funcs = config.active_node.n2r
        n2r_funcs.append(self)
        args = inspect.getargspec(func).args
        if args[0] != "t":
            raise Exception("The first parameter of a transfer function must be the time!")
        self.__neuron_params = args[1:]
        return self

    def replace_params(self):
        """
        Replaces strings to neuron references
        """
        for i in range(0, len(self.__neuron_params)):
            if type(self.__neuron_params[i]) == str:
                param_name = self.__neuron_params[i].lower()
                gid = None
                if param_name.startswith("neuron"):
                    gid = int(param_name[6:])
                elif param_name.startswith("n"):
                    gid = int(param_name[1:])
                self.__neuron_params[i] = ([gid], INeuronVoltmeter)

    def __repr__(self):
        return "{0} transfers to robot {1} {2} using {3}" \
            .format(self.__func, self.__main_robot_topic, self.__robot_topics, self.__neuron_params)

    def __run_0(self, t):
        assert callable(self.__func)
        return self.__func(t)

    def __run_1(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0])

    def __run_2(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0], self.__neuron_params[1])

    def __run_3(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0], self.__neuron_params[1], self.__neuron_params[2])

    def __run_4(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0], self.__neuron_params[1], self.__neuron_params[2],
                           self.__neuron_params[3])

    def __run_5(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0], self.__neuron_params[1], self.__neuron_params[2],
                           self.__neuron_params[3], self.__neuron_params[4])

    def __run_6(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0], self.__neuron_params[1], self.__neuron_params[2],
                           self.__neuron_params[3], self.__neuron_params[4], self.__neuron_params[5])

    def __run_7(self, t):
        assert callable(self.__func)
        return self.__func(t, self.__neuron_params[0], self.__neuron_params[1], self.__neuron_params[2],
                           self.__neuron_params[3], self.__neuron_params[4], self.__neuron_params[5],
                           self.__neuron_params[6])

    __run_list = [__run_0, __run_1, __run_2, __run_3, __run_4, __run_5, __run_6, __run_7]

    def run(self, t):
        return_value = Neuron2Robot.__run_list[len(self.__neuron_params)](self, t)
        if return_value is not None:
            self.__main_robot_topic.send_message(return_value)