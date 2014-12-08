"""
The mock implementation of the communication adapter to the world simulation
"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter, IRobotPublishedTopic, \
    IRobotSubscribedTopic

__author__ = 'GeorgHinkel'


class Roscore(object):
    """
    Represents a mock implementation of the message passing in roscore
    """
    def __init__(self):
        """
        Creates a new mock message passing
        """
        self._associations = []

    def connect(self, pub, sub):
        """
        Connect a publisher to a subscriber"
        """
        self._associations.append((pub, sub))

    def sendAll(self):
        """
        Send all present messages to the connected subscribers
        """
        for association in self._associations:
            data = association[0].sent
            for d in data:
                association[1].value = d


class MockRobotCommunicationAdapter(IRobotCommunicationAdapter):
    """
    Represents a mock implementation of the world simulation communication adapter
    """
    def __init__(self):
        """
        Creates a new mock robot adapter
        """
        super(MockRobotCommunicationAdapter, self).__init__()
        self.__name = None

    def initialize(self, name):
        """
        Initializes the mock with the given name
        :param name: The name
        """
        self.__name = name

    @property
    def name(self):
        """
        Gets the name of the mock
        """
        return self.__name

    def create_topic_publisher(self, topic, config):
        """
        Creates a publisher object for the given topic
        :param topic: The topic
        :param config: Additional configuration for the publisher
        :return: A publisher object
        """
        return MockPublishedTopic()

    def create_topic_subscriber(self, topic, config):
        """
        Creates the subscription object for the given topic
        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :return: A subscription object
        """
        return MockSubscribedTopic()

    def refresh_buffers(self, t):
        """
        Refreshes the subscribed topic buffers for the given simulation time
        :param t: The simulation time
        """
        pass


class MockPublishedTopic(IRobotPublishedTopic):
    """
    Represents a mock for a robot topic publisher
    """
    def __init__(self):
        """
        Initializes the new mock
        """
        self.__sent = []

    def send_message(self, value):
        """
        Send a message to the robot topic represented by this instance
        :param value: The message to be sent to the robot
        """
        self.__sent.append(value)

    @property
    def sent(self):
        """
        Gets a list of the messages sent to the mocked publisher
        """
        return self.__sent


class MockSubscribedTopic(IRobotSubscribedTopic):
    """
    Represents a mock for a robot topic subscriber
    """

    def __init__(self):
        """
        Created a new mock
        """
        self.__changed = False
        self.__value = None

    @property
    def changed(self):
        """
        Gets a value indicating whether the value of the subscribed topic has changed since the last
        time step
        """
        return self.__changed

    @property
    def value(self):
        """
        Gets the current value of the subscribed topic
        """
        return self.__value

    @value.setter
    def value(self, value):  # pylint: disable=W0221
        """
        Sets the current value of the subscribed topic
        :param value: The new current value
        """
        self.__changed = (value != self.__value)
        self.__value = value
