from robotsim.RobotInterface import IRobotControlAdapter

__author__ = 'NinoCauli'


class MockRobotControlAdapter(IRobotControlAdapter):
    
    def __init__(self):
        self.__timestamp = 0.0

    def initialize(self):
        pass

    def is_paused(self):
        return True

    def is_alive(self):
        return True

    def run_step(self, dt):
        self.__timestamp = self.__timestamp + (dt / 1000.0);
        return self.__timestamp

    def shutdown(self):
        pass
    