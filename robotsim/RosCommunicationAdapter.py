from robotsim.RobotInterface import IRobotCommunicationAdapter, Topic, IRobotSubscribedTopic, IRobotPublishedTopic
#import rospy
# TODO: implement RosAdapter actually using ROS
__author__ = 'GeorgHinkel'


class RosCommunicationAdapter(IRobotCommunicationAdapter):
    def initialize(self, name):
        #rospy.init_node(name)
        pass

    def create_topic_publisher(self, topic, config):
        return RosPublishedTopic(topic)

    def create_topic_subscriber(self, topic, config):
        return RosSubscribedTopic(topic)

    def is_alive(self):
        #return not rospy.is_shutdown()
        return True


class RosPublishedTopic(IRobotPublishedTopic):
    def __init__(self, topic):
        self.__lastSent = None
        assert isinstance(topic, Topic)
        #self.__pub = rospy.Publisher(topic.name, topic.type, queue_size=10)

    def send_message(self, value):
        if value != self.__lastSent:
            #self.__pub.publish(value)
            self.__lastSent = value


class RosSubscribedTopic(IRobotSubscribedTopic):
    def __init__(self, topic):
        self.__changed = False
        self.__value = None
        assert isinstance(topic, Topic)
        #self.__subscriber = rospy.Subscriber(topic.name, topic.type, self.__callback)

    def __callback(self, data):
        self.__changed = True
        self.__value = data

    def __get_changed(self):
        return self.__changed

    def __get_value(self):
        return self.__value

    changed = property(__get_changed)

    value = property(__get_value)