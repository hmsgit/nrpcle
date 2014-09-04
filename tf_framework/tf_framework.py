import inspect

from brainsim import PyNNCommunicationAdapter

from robotsim.RobotInterface import Topic, IRobotCommunicationAdapter
from brainsim.BrainInterface import NeuronReference, IBrainCommunicationAdapter, IFixedFrequencySpikeGenerator, \
    INeuronVoltmeter, IPoissonSpikeGenerator, IPatternSpikeGenerator, ISpikeRecorder
from robotsim import RosCommunicationAdapter


__author__ = 'GeorgHinkel'

voltmeter = INeuronVoltmeter
fixed_frequency = IFixedFrequencySpikeGenerator
poisson = IPoissonSpikeGenerator
pattern = IPatternSpikeGenerator
recorder = ISpikeRecorder


def initialize(name):
    """
    Initializes and starts the TF node
    """
    active_node.initialize(name)


def spike(neuron_name):
    """
    Creates a reference to the given neuron
    :param neuron_name the id of the referenced neuron
    """
    return spikes(neuron_name, 1)


def spikes(neuron_name, n):
    """
    Creates a reference to a sequence of neurons
    :param neuron_name: the id of the first referenced neuron
    :param n: the amount of referenced spikes
    """
    return NeuronReference(neuron_name, n)


def send_robot(topic, value):
    """
    Send data to the given robot topic
    :param topic: The robot topic
    :param value: The values sent to the robot
    """
    print("Sending ", value, " to robot ", topic)


def set_nest_adapter(nest_adapter):
    """
    Sets the brainsim adapter. Must be executed before tf node initialization
    :param nest_adapter: The brainsim adapter
    """
    active_node.nest_adapter = nest_adapter


def set_robot_adapter(robot_adapter):
    """
    Sets the robot adapter. Must be run before tf node initialization
    :param robot_adapter: The robot adapter
    """
    active_node.robot_adapter = robot_adapter


class TransferFunctionNode(object):
    """
    Represents a transfer functions node
    """

    def __init__(self):
        self.__n2r = []
        self.__r2n = []
        self.__robotAdapter = None
        self.__nestAdapter = None
        self.__initialized = False

    def __get_n2r(self):
        return self.__n2r

    def __get_r2n(self):
        return self.__r2n

    def run_n2r(self, t):
        for _n2r in self.__n2r:
            assert isinstance(_n2r, Neuron2Robot)
            _n2r.run()

    def run_r2n(self, t):
        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            _r2n.run()

    def initialize(self, name):
        if self.__nestAdapter is None:
            self.__nestAdapter = PyNNCommunicationAdapter()

        if self.__robotAdapter is None:
            self.__robotAdapter = RosCommunicationAdapter()

        assert isinstance(self.__nestAdapter, IBrainCommunicationAdapter)
        assert isinstance(self.__robotAdapter, IRobotCommunicationAdapter)

        self.__nestAdapter.initialize(name)
        self.__robotAdapter.initialize(name)

        for _n2r in self.__n2r:
            assert isinstance(_n2r, Neuron2Robot)
            _n2r.replace_params()

            _n2r.main_robot_topic = self.__robotAdapter.register_publish_topic(_n2r.main_robot_topic)
            for i in range(0, len(_n2r.robot_topics)):
                _n2r.robot_topics[i] = self.__robotAdapter.register_publish_topic(_n2r.robot_topics[i])

            for i in range(0, len(_n2r.neuron_params)):
                item = _n2r.neuron_params[i]
                _n2r.neuron_params[i] = self.__nestAdapter.register_consume_spikes(item[0], item[1])

        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            _r2n.check_params()

            for i in range(0, len(_r2n.params)):
                item = _r2n.params[i]
                if isinstance(item, Topic):
                    _r2n.params[i] = self.__robotAdapter.register_subscribe_topic(item)
                else:
                    _r2n.params[i] = self.__nestAdapter.register_generate_spikes(item[0], item[1])

    def __get_robot_adapter(self):
        return self.__robotAdapter

    def __set_robot_adapter(self, robot_adapter):
        if self.__initialized:
            raise Exception("Cannot exchange robot adapter after node has been initialized!")
        else:
            assert isinstance(robot_adapter, IRobotCommunicationAdapter)
            self.__robotAdapter = robot_adapter

    def __get_nest_adapter(self):
        return self.__nestAdapter

    def __set_nest_adapter(self, nest_adapter):
        if self.__initialized:
            raise Exception("Cannot exchange brainsim adapter after node has been initialized!")
        else:
            assert isinstance(nest_adapter, IBrainCommunicationAdapter)
            self.__nestAdapter = nest_adapter

    n2r = property(__get_n2r)

    r2n = property(__get_r2n)

    robot_adapter = property(__get_robot_adapter, __set_robot_adapter)

    nest_adapter = property(__get_nest_adapter, __set_nest_adapter)


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
        assert isinstance(value, NeuronReference)
        # TODO: handle the case when the device_type is not a type
        #assert isinstance(device_type, type)
        self.__key = key
        self.__value = value
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
        n2r_funcs = active_node.n2r
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
                self.__neuron_params[i] = (spike(self.__neuron_params[i]), INeuronVoltmeter)

    def __repr__(self):
        return "{0} transfers to robot {1} {2} using {3}" \
            .format(self.__func, self.__main_robot_topic, self.__robot_topics, self.__neuron_params)

    def run(self):
        pass


class MapRobotParameter(object):
    """
    Class to map parameters to robot topics
    """

    def __init__(self, key, value):
        """
        Maps a parameter to a robot topic
        :param key: the name of the parameter
        :param value: the value for the parameter
        """
        assert isinstance(value, Topic)
        self.__key = key
        self.__value = value

    def __call__(self, r2n):
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

    def __init__(self):
        """
        Creates a new transfer function from robots to neurons
        """
        self.__func = None
        self.__params = []

    def __get_params(self):  # -> list:
        return self.__params

    params = property(__get_params)

    def __call__(self, func):
        self.__func = func
        r2n_funcs = active_node.r2n
        r2n_funcs.append(self)
        args = inspect.getargspec(func).args
        if args[0] != "t":
            raise Exception("The first parameter of a transfer function must be the time!")
        self.__params = args[1:]
        return self

    def __repr__(self):
        return "{0} transfers to neurons {1}".format(self.__func, self.__params)

    def check_params(self):
        """
        Checks whether all parameters have been mapped to a robot topic
        :exception Exception if a parameter was not mapped to a robot topic
        """
        for topic in self.__params:
            if type(topic) == str:
                raise Exception("Parameter ", topic, " was not mapped to a robot topic")

    def run(self):
        pass


active_node = TransferFunctionNode()