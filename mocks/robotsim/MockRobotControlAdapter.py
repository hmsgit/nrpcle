from robotsim.RobotInterface import IRobotControlAdapter

__author__ = 'NinoCauli'


class MockRobotControlAdapter(IRobotControlAdapter):
    
    def __init__(self):
        self.__sim_time = 0.0
        self.__time_step = 0.001

    def initialize(self):
        pass

    def get_time_step(self):      
        return self.__time_step

    def set_time_step(self, time_step):
        self.__time_step = time_step       
        return True

    def is_paused(self):
        return True

    def is_alive(self):
        return True

    def run_step(self, dt):
        self.__sim_time = self.__sim_time + (dt * self.__time_step)
        return self.__sim_time

    def shutdown(self):
        pass
    