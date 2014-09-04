from robotsim.RobotInterface import IRobotCommunicationAdapter, Topic

__author__ = 'GeorgHinkel'


class MockRobotCommunicationAdapter(IRobotCommunicationAdapter):
    def initialize(self, name):
        pass

    def create_topic_publisher(self, topic):
        return MockPublishedTopic(topic)

    def create_topic_subscriber(self, topic):
        return MockSubscribedTopic(topic)

    def is_alive(self):
        return True


class MockPublishedTopic(object):
    def __init__(self, topic):
        self.__lastSent = None
        self.__topic = topic

    def send_message(self, value):
        print("Sending ", value, " on topic ", self.__topic)


class MockSubscribedTopic(object):
    def __init__(self, topic):
        self.__changed = False
        self.__value = None
        assert isinstance(topic, Topic)

    def get_changed(self):
        return self.__changed

    def get_value(self):
        return self.__value

    changed = property(get_changed)

    value = property(get_value)