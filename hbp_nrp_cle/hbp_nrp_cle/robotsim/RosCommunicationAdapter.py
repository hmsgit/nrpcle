"""
Represents an implementation of the robot communication adapter actually
using ROS
"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter, \
    Topic, PreprocessedTopic, IRobotSubscribedTopic, IRobotPublishedTopic
import rospy
# import std_msgs.msg

__author__ = 'GeorgHinkel'


class RosCommunicationAdapter(IRobotCommunicationAdapter):
    """
    Represents a robot communication adapter actually using ROS
    """

    def initialize(self, name):
        """
        Initializes this robot communication adapter
        :param name: The name of this node
        """
        try:
            rospy.init_node(name)
        except rospy.exceptions.ROSException:
            print "ROS node already initialized"

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
        :param t: The world simulation time
        """
        for key in self.subscribed_topics:
            self.subscribed_topics[key].reset_changed()


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
        print("ros publisher created: topic.name = ", topic.name,
              " topic.type ", topic.topic_type)
        self.__pub = rospy.Publisher(topic.name, topic.topic_type,
                                     queue_size=10)

    def send_message(self, value):
        """
        Sends a message
        :param value: The message to be sent
        """
        # if value != self.__lastSent:
        self.__pub.publish(value)
        self.__lastSent = value
        # print("ros message published: topic name = ", Topic.name,
        # " topic value = ", value)


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

    def _callback(self, data):
        """
        This method is called whenever new data is available from ROS
        :param data: The incoming data on this topic
        """
        print("ros subscriber callback")
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
        """
        self.reset_changed()
        return self


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
