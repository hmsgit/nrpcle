"""
Represents an implementation of the robot communication adapter actually
using ROS
"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter, \
    Topic, PreprocessedTopic, IRobotSubscribedTopic, IRobotPublishedTopic
import rospy
import logging

logger = logging.getLogger(__name__)

__author__ = 'GeorgHinkel'


class RosCommunicationAdapter(IRobotCommunicationAdapter):
    """
    Represents a robot communication adapter actually using ROS
    """

    def __init__(self):
        IRobotCommunicationAdapter.__init__(self)

    def initialize(self, name):
        """
        Initializes this robot communication adapter

        :param name: The name of this node
        """
        try:
            rospy.init_node(name)
            logger.info("Robot comunication adapter initialized")
        except rospy.exceptions.ROSException:
            logger.warn("ROS node already initialized")

    def create_topic_publisher(self, topic, config):
        """
        Creates a publisher object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the publisher
        :return: A publisher object
        """
        if isinstance(topic, PreprocessedTopic):
            return RosPublishedPreprocessedTopic(topic)
        return RosPublishedTopic(topic)

    def create_topic_subscriber(self, topic, config):
        """
        Creates the subscription object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :return: A subscription object
        """
        if isinstance(topic, PreprocessedTopic):
            return RosSubscribedPreprocessedTopic(topic)
        return RosSubscribedTopic(topic)

    @property
    def is_alive(self):  # pylint: disable=R0201
        """
        Gets a value indicating whether the robot simulation is still alive
        """
        return not rospy.is_shutdown()

    def refresh_buffers(self, t):
        """
        Resets the changed bit for all subscribers

        :param t: The world simulation time in milliseconds
        """
        for subscriber in self.subscribed_topics:
            subscriber.reset_changed()

    def shutdown(self):
        """
        Closes any connections created by the adapter
        """
        for publisher in self.published_topics:
            publisher.unregister()
        for subscriber in self.subscribed_topics:
            subscriber.unregister()


class RosPublishedTopic(IRobotPublishedTopic):
    """
    Represents a robot topic publisher actually using ROS
    """
    def __init__(self, topic):
        """
        Creates a new robot topic publisher

        :param topic: The topic where data should be sent to
        """
        self.__lastSent = None
        assert isinstance(topic, Topic)
        logger.info("ROS publisher created: topic name = %s, topic type = %s",
                    topic.name, topic.topic_type)
        self.__pub = rospy.Publisher(topic.name, topic.topic_type, queue_size=10)

    def send_message(self, value):
        """
        Sends a message

        :param value: The message to be sent (the type must match the topic type)
        """
        # if value != self.__lastSent:
        if self.__pub is not None:
            self.__pub.publish(value)
            self.__lastSent = value
            logger.debug("ROS message published: topic value = %s",
                         value)
        else:
            logger.error("Trying to publish messages on an unregistered topic")

    def unregister(self):
        """
        Unregister the Topic. After this call, nobody can publish
        anymore.
        """
        self.__pub.unregister()
        self.__pub = None


class RosPublishedPreprocessedTopic(RosPublishedTopic):
    """
    Represents a robot topic publisher actually using ROS
    """
    def __init__(self, topic):
        """
        Creates a new robot topic publisher

        :param topic: The topic where data should be sent to
        """
        super(RosPublishedPreprocessedTopic, self).__init__(topic)
        assert isinstance(topic, PreprocessedTopic)
        self.__pre_processor = topic.pre_processor

    def send_message(self, value):
        """
        Sends a message
        :param value: The message to be sent
        """
        to_send = self.__pre_processor(value)
        super(RosPublishedPreprocessedTopic, self).send_message(to_send)


class RosSubscribedTopic(IRobotSubscribedTopic):
    """
    Represents a robot topic subscriber actually using ROS
    """

    def __init__(self, topic):
        """
        Initializes a new subscriber for the given topic

        :param topic: The topic that is subscribed
        """
        self.__changed = False
        self.__value = None
        assert isinstance(topic, Topic)
        self.__subscriber = rospy.Subscriber(topic.name, topic.topic_type,
                                             self._callback)
        logger.info("ROS subscriber created: topic name = %s, topic type = %s",
                     topic.name, topic.topic_type)

    def _callback(self, data):
        """
        This method is called whenever new data is available from ROS

        :param data: The incoming data on this topic
        """
        logger.debug("ROS subscriber callback")
        self.__changed = True
        self.__value = data

    @property
    def changed(self):
        """
        Indicates whether the current value of this subscriber has changed
        since the last iteration
        """
        return self.__changed

    def reset_changed(self):
        """
        Resets the changed status of the current subscriber
        """
        self.__changed = False

    @property
    def value(self):
        """
        Gets the last value received by this ROS subscribed topic
        """
        return self.__value

    def reset(self, transfer_function_manager):
        """
        Gets a reset subscriber

        :param transfer_function_manager: The transfer function manager in which the subscribed
         topic is contained
        """
        self.reset_changed()
        self.__value = None
        return self

    def unregister(self):
        """
        Detaches the subscribed topic from ROS
        """
        self.__subscriber.unregister()
        self.__subscriber = None


class RosSubscribedPreprocessedTopic(RosSubscribedTopic):
    """
    Represents a robot topic subscriber with a preprocessor
    """

    def __init__(self, topic):
        """
        Creates a new preprocessing topic subscriber

        :param topic: The topic that is subscribed
        """
        super(RosSubscribedPreprocessedTopic, self).__init__(topic)
        assert isinstance(topic, PreprocessedTopic)
        self.__pre_processor = topic.pre_processor

    def _callback(self, data):
        """
        This method is called whenever new data is available from ROS

        :param data: The incoming data
        """
        pre_processed = self.__pre_processor(data)
        super(RosSubscribedPreprocessedTopic, self)._callback(pre_processed)
