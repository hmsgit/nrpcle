"""
This module contains the mock implementation of the robot control adapter
"""

from python_cle.robotsim.RobotInterface import IRobotControlAdapter

__author__ = 'NinoCauli'


class MockRobotControlAdapter(IRobotControlAdapter):
    """
    Represents a mock implementation of the robot control adapter
    """
    def __init__(self):
        """
        Creates a new robot control adapter mock
        """
        self.__sim_time = 0.0
        self.__time_step = 0.001

    def initialize(self):
        """
        Initializes the world simulation control adapter
        """
        pass

    @property
    def time_step(self):
        """
        Gets the physics simulation time step in seconds
        :param dt: The physics simulation time step in seconds
        :return: The physics simulation time step in seconds
        """
        return self.__time_step

    def set_time_step(self, time_step):
        """
        Sets the physics simulation time step in seconds
        :param dt: The physics simulation time step in seconds
        :return: True, if the physics simulation time step is updated, otherwise False
        """
        self.__time_step = time_step
        return True

    @property
    def is_paused(self):
        """
        Queries the current status of the physics simulation
        :return: True, if the physics simulation is paused, otherwise False
        """
        return True

    @property
    def is_alive(self):
        """
        Queries the current status of the world simulation
        :return: True, if the world simulation is alive, otherwise False
        """
        return True

    def run_step(self, dt):
        """
        Runs the world simulation for the given CLE time step in seconds
        :param dt: The CLE time step in seconds
        :return: Updated simulation time, otherwise -1
        """
        if dt % self.__time_step == 0:
            self.__sim_time = self.__sim_time + dt
            simTime = self.__sim_time
        else:
            simTime = -1
            raise ValueError("dt is not multiple of the physics time step")
        return simTime

    def shutdown(self):
        """
        Shuts down the world simulation
        """
        pass
