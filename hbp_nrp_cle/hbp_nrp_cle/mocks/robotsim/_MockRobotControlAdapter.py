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
This module contains the mock implementation of the robot control adapter
"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotControlAdapter
from concurrent.futures import Future
import math

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
        if math.fmod(dt, self.__time_step) < 1e-10:
            self.__sim_time = self.__sim_time + dt
            simTime = self.__sim_time
            return simTime
        else:
            raise ValueError("dt is not multiple of the physics time step")

    def run_step_async(self, dt):
        """
        Runs the world simulation for the given CLE time step in seconds in an asynchronous manner.

        :param dt: The CLE time step in seconds
        :return: a Future for the result or potential exceptions of the execution
        """
        future = Future()
        future.set_result(self.run_step(dt))
        future.start = 0
        future.end = 42
        return future

    def shutdown(self):
        """
        Shuts down the world simulation
        """
        pass

    def reset(self):
        """
        Resets the physics simulation
        """
        self.__sim_time = 0.0

    def unpause(self):
        """
        Unpaused the physics
        """
        pass

    def pause(self):
        """
        Pause the physics
        """
        pass

    def reset_world(self, models, lights):
        """
        Resets the world excluding the robot
        :param models: A dictionary containing pairs model_name: model_sdf.
        :param lights: A dictionary containing pairs light_name: light sdf.
        """
        pass
