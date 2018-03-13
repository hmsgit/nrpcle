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
Defines an API extending the functionality available to the TFs
"""

from cle_ros_msgs.srv import ResetSimulationRequest


class TfApi(object):
    """
    Tf API
    """

    __ros_cle_server = None

    def set_ros_cle_server(self, ros_cle_server):
        """
        The reset is NRP code, that is not known by the CLE
        Exposing the ros_cle_server here solves this problem

        A refactoring task [ADD-ID_HERE] exists to move the TF instanciation from CLE.tf_framework
        into the ROSCLEServer and the GLOBALS code required by the TF code exec
        """
        self.__ros_cle_server = ros_cle_server

    def reset_robot(self):
        """
        Resets the robot position
        """
        msg = ResetSimulationRequest()
        msg.reset_type = ResetSimulationRequest.RESET_ROBOT_POSE
        self.__ros_cle_server.reset_simulation(msg)
