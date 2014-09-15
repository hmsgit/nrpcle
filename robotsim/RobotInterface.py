__author__ = 'GeorgHinkel'


class Topic(object):
    """
    Represents a reference to a robot topic
    """

    def __init__(self, name, topic_type):  # -> None:
        """
        Create a new robot type reference
        :param name: the name of the topic
        :param topic_type: the type of the topic
        """
        self.__name = name
        self.__type = topic_type

    def __get_name(self):  # -> str:
        return self.__name

    name = property(__get_name)

    def __get_type(self):  # -> type:
        return self.__type

    type = property(__get_type)

    def __repr__(self):  # -> None:
        return self.__name + " : " + self.__type.__name__


class IRobotPublishedTopic(object):
    """
    Represents a communication object for a published robot topic
    """
    def send_message(self, value):  # -> None:
        """
        Send a message to the robot topic represented by this instance
        :param value: The message to be sent to the robot
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotSubscribedTopic(object):
    """
    Represents a communication object for a subscribed robot topic
    """

    def __get_changed(self):  # -> bool:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __get_value(self):  # -> object:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    changed = property(__get_changed)

    value = property(__get_value)


class IRobotCommunicationAdapter(object):
    """
    Represents the communication adapter to the robot
    """
    def __init__(self):  # -> None:
        self.__published_topics = {}
        self.__subscribed_topics = {}

    @property
    def published_topics(self):  # -> list:
        """
        Gets the published topics for the robot communication adapter
        :return: A hash table of the communication adapters published topics
        """
        return self.__published_topics

    @property
    def subscribed_topics(self):  # -> list:
        """
        Gets the subscribed topics for the robot communication adapter
        :return: A hash table of the communication adapters subscribed topics
        """
        return self.__subscribed_topics

    def register_subscribe_topic(self, topic, **kwargs):  # -> IRobotSubscribedTopic:
        """
        Requests a subscription object for the given topic
        :param topic: The topic that should be subscribed
        :param kwargs: Additional configuration parameters
        :return: A subscription object that holds the current data
        """
        if not topic in self.__subscribed_topics:
            subscriber = self.create_topic_subscriber(topic, kwargs)
            self.__subscribed_topics[topic] = subscriber
            return subscriber
        else:
            return self.__subscribed_topics[topic]

    def register_publish_topic(self, topic, **kwargs):  # -> IRobotPublishedTopic:
        """
        Requests a publisher object for the given topic
        :param topic: The topic for which to create a publisher
        :param kwargs: Additional configuration parameters
        :return: A publisher communication object
        """
        if not topic in self.__published_topics:
            publisher = self.create_topic_publisher(topic, kwargs)
            self.__published_topics[topic] = publisher
            return publisher
        else:
            return self.__published_topics[topic]

    def create_topic_subscriber(self, topic, config):  # -> IRobotSubscribedTopic:
        """
        Creates the subscription object for the given topic
        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :return: A subscription object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def create_topic_publisher(self, topic, config):  # -> IRobotPublishedTopic:
        """
        Creates a publisher object for the given topic
        :param topic: The topic
        :param config: Additional configuration for the publisher
        :return: A publisher object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self, name):  # -> None:
        """
        Initializes the robot adapter
        :param name: The name of the node
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def refresh_buffers(self, t):  # -> None:
        """
        Refreshes the subscribed topic buffers for the given simulation time
        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotControlAdapter(object):
    """
    Represents a control adapter for the world simulation
    """
    def is_alive(self):  # -> bool:
        """
        Queries the current status of the world simulation
        :return: True, if the world simulation is alive, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
    
    def is_paused(self):  # -> bool:
        """
        Queries the current status of the physics simulation
        :return: True, if the physics simulation is paused, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self):  # -> None:
        """
        Initializes the world simulation control adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def run_step(self, dt):  # -> None:
        """
        Runs the world simulation for the given time step
        :param dt: The time step
        :return: Updated simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def shutdown(self):  # -> None:
        """
        Shuts down the world simulation
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")