from python_cle.robotsim.RobotInterface import IRobotCommunicationAdapter, Topic

__author__ = 'GeorgHinkel'


class MockRobotCommunicationAdapter(IRobotCommunicationAdapter):
    def initialize(self, name):
        pass

    def create_topic_publisher(self, topic, config):
        return MockPublishedTopic()

    def create_topic_subscriber(self, topic, config):
        return MockSubscribedTopic()

    def is_alive(self):
        return True

    def refresh_buffers(self, t):
        pass


class MockPublishedTopic(object):
    def __init__(self):
        self.__sent = []

    def send_message(self, value):
        self.__sent.append(value)

    @property
    def sent(self):
        return self.__sent


class MockSubscribedTopic(object):
    def __init__(self):
        self.__changed = False
        self.__value = None

    def get_changed(self):
        return self.__changed

    def get_value(self):
        return self.__value

    def set_value(self, value):
        self.__changed = value != self.__value
        self.__value = value

    changed = property(get_changed)

    value = property(get_value, set_value)
