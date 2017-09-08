# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
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
        """
        Create a new RosCommunicationAdapter
        """
        IRobotCommunicationAdapter.__init__(self)

    def initialize(self, name):
        """
        Initializes this robot communication adapter

        :param name: The name of this node
        """
        try:
            rospy.init_node(name)
            logger.info("Robot communication adapter initialized")
        except rospy.exceptions.ROSException:
            logger.warn("ROS node already initialized with another name")

    def create_topic_publisher(self, topic, config):
        """
        Creates a publisher object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the publisher
        :return: A publisher object
        """
        if isinstance(topic, PreprocessedTopic):
            return RosPublishedPreprocessedTopic(topic, config.get('queue_size', 10))
        return RosPublishedTopic(topic, config.get('queue_size', 10))

    def create_topic_subscriber(self, topic, config):
        """
        Creates the subscription object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :return: A subscription object
        """
        if isinstance(topic, PreprocessedTopic):
            return RosSubscribedPreprocessedTopic(topic, config.get('initial_value', None))
        return RosSubscribedTopic(topic, config.get('initial_value', None))

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
            publisher._unregister()  # pylint: disable=protected-access
        for subscriber in self.subscribed_topics:
            subscriber._unregister()  # pylint: disable=protected-access


class RosPublishedTopic(IRobotPublishedTopic):
    """
    Represents a robot topic publisher actually using ROS
    """
    def __init__(self, topic, queue_size):
        """
        Creates a new robot topic publisher

        :param topic: The topic where data should be sent to
        :param queue_size: The queue size for the publisher
        """
        self.__lastSent = None
        assert isinstance(topic, Topic)
        logger.info("ROS publisher created: topic name = %s, topic type = %s",
                    topic.name, topic.topic_type)
        self.__pub = rospy.Publisher(topic.name, topic.topic_type, queue_size=queue_size)

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

    def _unregister(self):
        """
        Unregister the Topic. After this call, nobody can publish
        anymore.
        """
        if self.__pub:
            self.__pub.unregister()
            self.__pub = None


class RosPublishedPreprocessedTopic(RosPublishedTopic):
    """
    Represents a robot topic publisher actually using ROS
    """
    def __init__(self, topic, queue_size):
        """
        Creates a new robot topic publisher

        :param topic: The topic where data should be sent to
        :param queue_size: The queue size for the publisher
        """
        super(RosPublishedPreprocessedTopic, self).__init__(topic, queue_size)
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

    def __init__(self, topic, initial_value):
        """
        Initializes a new subscriber for the given topic

        :param topic: The topic that is subscribed
        :initial_value: The initial value for the subscriber
        """
        self.__changed = False
        self.__value = initial_value
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

    def _unregister(self):
        """
        Detaches the subscribed topic from ROS
        """
        if self.__subscriber:
            self.__subscriber.unregister()
            self.__subscriber = None


class RosSubscribedPreprocessedTopic(RosSubscribedTopic):
    """
    Represents a robot topic subscriber with a preprocessor
    """

    def __init__(self, topic, initial_value):
        """
        Creates a new preprocessing topic subscriber

        :param topic: The topic that is subscribed
        :param initial_value: The initial value for the subscriber
        """
        super(RosSubscribedPreprocessedTopic, self).__init__(topic, initial_value)
        assert isinstance(topic, PreprocessedTopic)
        self.__pre_processor = topic.pre_processor

    def _callback(self, data):
        """
        This method is called whenever new data is available from ROS

        :param data: The incoming data
        """
        pre_processed = self.__pre_processor(data)
        super(RosSubscribedPreprocessedTopic, self)._callback(pre_processed)
