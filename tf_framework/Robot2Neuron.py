__author__ = 'GeorgHinkel'

from robotsim.RobotInterface import Topic
from . import config

import inspect


class MapRobotParameter(object):
    """
    Class to map parameters to robot topics
    """

    def __init__(self, key, value):  # -> None:
        """
        Maps a parameter to a robot topic
        :param key: the name of the parameter
        :param value: the value for the parameter
        """
        assert isinstance(value, Topic)
        self.__key = key
        self.__value = value

    def __call__(self, r2n):  # -> Robot2Neuron:
        """
        Applies the parameter mapping to the given transfer function
        """
        assert isinstance(r2n, Robot2Neuron)
        topics = r2n.params
        for i in range(0, len(topics)):
            if topics[i] == self.__key:
                topics[i] = self.__value
                return r2n
        raise Exception("Could not map parameter as no parameter with the given name exists")


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
        self.__params = args[1:]
        return self

    def __repr__(self):  # -> str:
        return "{0} transfers to neurons {1}".format(self.__func, self.__params)

    def check_params(self):  # -> None:
        """
        Checks whether all parameters have been mapped to a robot topic
        :exception Exception if a parameter was not mapped to a robot topic
        """
        for topic in self.__params:
            if type(topic) == str:
                raise Exception("Parameter ", topic, " was not mapped to a robot topic")

    # The __run_i methods run the transfer function with i arguments. The reason why several of these methods are
    # required here is that Python does not offer a way to call a method with a number of arguments unknown at
    # compile time
    # Thus, the framework currently supports transfer functions with at most 8 parameters where the first parameter
    # is the time

    def __run_0(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t)

    def __run_1(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0])

    def __run_2(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0], self.__params[1])

    def __run_3(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0], self.__params[1], self.__params[2])

    def __run_4(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0], self.__params[1], self.__params[2],
                           self.__params[3])

    def __run_5(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0], self.__params[1], self.__params[2],
                           self.__params[3], self.__params[4])

    def __run_6(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0], self.__params[1], self.__params[2],
                           self.__params[3], self.__params[4], self.__params[5])

    def __run_7(self, t):  # -> object:
        assert callable(self.__func)
        return self.__func(t, self.__params[0], self.__params[1], self.__params[2],
                           self.__params[3], self.__params[4], self.__params[5],
                           self.__params[6])

    __run_list = [__run_0, __run_1, __run_2, __run_3, __run_4, __run_5, __run_6, __run_7]

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulated time
        :param t: The simulation time
        """
        Robot2Neuron.__run_list[len(self.__params)](self, t)