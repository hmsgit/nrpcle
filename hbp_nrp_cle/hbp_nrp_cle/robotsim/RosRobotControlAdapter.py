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
#!/usr/bin/env python
"""Implementation of the robot control adapter using ros and gazebo"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotControlAdapter
import logging
import time


logger = logging.getLogger(__name__)
# Info messages sent to this logger will be forwarded as notifications
user_notifications_logger = logging.getLogger('hbp_nrp_cle.user_notifications')

__author__ = 'LorenzoVannucci'


def unused(_):
    """
    Marks an argument as unused
    """
    pass


class RosRobotControlAdapter(IRobotControlAdapter):
    """
    Represents a robot adapter that basically does nothing
    """

    def __init__(self):
        self.__is_initialized = False
        self.__time = 0.0

    def initialize(self):
        """
        Initializes the robot control adapter
        """
        if not self.__is_initialized:
            self.__is_initialized = True

        logger.info("Robot control adapter initialized")
        return self.__is_initialized

    @property
    def time_step(self):
        """
        This property has no significance for the robot and should not be used
        """
        return 0.0

    def set_time_step(self, time_step):
        """
        This function has no significance for the robot and should not be used
        """
        unused(time_step)
        return True

    @property
    def is_paused(self):
        """
        Queries the current status of the robotic hardware

        :return: always False, as the robot cannot be paused
        """
        return False

    @property
    def is_alive(self):
        """
        Queries the current status of the robotic hardware

        :return: always True, as the robot should always be alive
        """
        logger.debug("The robot should always be alive")
        return True

    def run_step_async(self, dt):
        """
        This function has no significance for the robot, and it just complies with the APIs

        :param dt: The CLE time step in seconds
        """

        # the CLE expects a future
        class DummyFuture(object):
            """
            A dummy future that complies with the APIs
            """

            def __init__(self, start):
                self.start = start
                self.end = None

            def result(self):
                """
                Return this, to comply with the APIs
                """
                return self

        ret = DummyFuture(time.time())
        self.run_step(dt)
        ret.end = time.time()
        return ret

    def run_step(self, dt):
        """
        This function has no significance for the robot, and it just complies with the APIs

        :param dt: The CLE time step in seconds
        """
        self.__time = self.__time + dt
        return self.__time

    def shutdown(self):
        """
        The robot cannot be shut down by this
        """
        logger.info("Please shutdown the robot")

    def reset(self):
        """
        The robot cannot be reset by this and it should be reset manually
        """
        self.__time = 0.0
        logger.info("Please reset the robot")

    def reset_world(self, models, lights):
        """
        The environment should be reset by hand
        """
        unused(models)
        unused(lights)
        logger.info("Please reset the environment around the robot")
