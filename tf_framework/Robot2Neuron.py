__author__ = 'GeorgHinkel'

from robotsim.RobotInterface import Topic
from . import config

import inspect


class MapRobotParameter(object):
    """
    Class to map parameters to robot topics
    """

    def __init__(self, key, value, **kwargs):  # -> None:
        """
        Maps a parameter to a robot topic
        :param key: the name of the parameter
        :param value: the value for the parameter
        :param kwargs: Additional configuration parameters
        """
        assert isinstance(value, Topic)
        self.__key = key
        self.__value = value
        self.__config = kwargs

    def __call__(self, r2n):  # -> Robot2Neuron:
        """
        Applies the parameter mapping to the given transfer function
        """
        assert isinstance(r2n, Robot2Neuron)
        topics = r2n.params
        for i in range(0, len(topics)):
            if topics[i] == self.__key:
                topics[i] = self
                return r2n
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

class Robot2Neuron(object):
    """
    Represents a transfer function from robot topics to neurons
    """

    def __init__(self):  # -> None:
        """
        Creates a new transfer function from robots to neurons
        """
        self.__func = None
        self.__params = []

    def __get_params(self):  # -> list:
        return self.__params

    # Gets the parameters of the current transfer function
    params = property(__get_params)

    def __call__(self, func):  # -> Robot2Neuron:
        """
        Attaches the given function to the current transfer function object
        :param func: The function implementing the transfer function
        :return: The transfer function object
        """
        self.__func = func
        r2n_funcs = config.active_node.r2n
        r2n_funcs.append(self)
        args = inspect.getargspec(func).args
        if args[0] != "t":
            raise Exception("The first parameter of a transfer function must be the time!")
        self.__params = list(args)
        return self

    def __repr__(self):  # pragma: no cover
        return "{0} transfers to neurons {1}".format(self.__func, self.__params)

    def check_params(self):  # -> None:
        """
        Checks whether all parameters have been mapped to a robot topic
        :exception Exception if a parameter was not mapped to a robot topic
        """
        for topic in self.__params:
            if topic != "t" and type(topic) == str:
                raise Exception("Parameter ", topic, " was not mapped to a robot topic")


    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulated time
        :param t: The simulation time
        """
        self.__params[0] = t
        self.__func(*self.__params)