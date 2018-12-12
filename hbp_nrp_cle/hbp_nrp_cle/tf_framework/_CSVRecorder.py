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
This module contains the mapping of a CSV recorder object to input parameters
"""
__author__ = 'Daniel Peppicelli, Georg Hinkel'

from ._MappingSpecification import ParameterMappingSpecification
from ._CleanableTransferFunctionParameter import ICleanableTransferFunctionParameter

import logging

logger = logging.getLogger(__name__)


class MapCSVRecorder(ParameterMappingSpecification):
    """
    Class to map a CSV recorder object to transfer function parameters
    """

    def __init__(self, parameter_name, filename, headers, erase_on_reset=False):
        """
        Maps a parameter to a variable in the specified scope (per-default: the transfer function)
        and if the variable does not yet exist initializes it with the provided value.
        A transfer function using it could look like this:

        @nrp.MapRobotSubscriber("joint_state",
                                Topic('/joint_states',
                                sensor_msgs.msg.JointState))
        @nrp.MapCSVRecorder("recorder",
                            filename = "all_joints_positions.csv",
                            headers=["Name", "time", "Position"])
        @nrp.Robot2Neuron()
        def joint_state_monitor(t, joint_state, recorder):
            for i in range(0, len(joint_state.value.name)):
                recorder.record_entry(joint_state.value.name[i], t, joint_state.value.position[i])

        :param parameter_name: the name of the parameter
        :param filename: the name of the file to write
        :param headers: An array of string containing the name of the columns that will be written
        in the CSV file
        :param erase_on_reset: A value indicating whether the csv recorder should erase its contents
        when the simulation is reset
        """
        super(MapCSVRecorder, self).__init__(parameter_name)
        self.filename = filename
        self.headers = headers
        self.__erase_on_reset = erase_on_reset

    def create_adapter(self, transfer_function_manager):  # pylint: disable=unused-argument
        """
        Replaces the current mapping operator with the mapping result

        :return: A ready to use CSVRecorder object
        """
        return CSVRecorder(self.filename, self.headers, self.__erase_on_reset)

    def create_tf(self):
        """
        Creates a TF in case the TF specification has been omitted
        """
        from hbp_nrp_cle.tf_framework._Neuron2Robot import Neuron2Robot
        return Neuron2Robot()


class CSVRecorder(ICleanableTransferFunctionParameter):
    """
    Record value and memory and dump them to a temporary file when asked to.
    """

    def __init__(self, filename, headers, erase_on_reset=False):
        """
        Constructor. Pretty straightforward,

        :param string filename: the filename to save to.
        :param string[] headers: the name of the columns.
        :param bool erase_on_reset: A value indicating whether the csv recorder should
        erase its contents when the simulation is reset
        """
        self.__filename = filename
        self.__reset_since_last_record = False
        headers = [str(header) + ',' for header in headers]
        self.__headers = [headers + ['Simulation_reset\n']]
        self.__headers = [
            item for sublist in self.__headers for item in sublist]
        self.__values = []
        self.__erase_on_reset = erase_on_reset

    def record_entry(self, *values):
        """
        Record the values provided (in memory) and creates the appropriate
        array to add to the values buffer

        :param string[] values : Values to record
        """
        # listify the values since they are tuples
        values = list(values)
        if self.__reset_since_last_record:
            values.append('RESET')

        # add a comma
        values = [str(val) + ',' for val in values]

        # in the last value we replace the comma with a newline
        values[-1] = values[-1].replace(',', '\n') if values[-1] else ""

        self.__reset_since_last_record = False
        self.__values.append(values)

    def get_csv_recorder_name(self):
        """
        Returns the CSV recorder filename

        :return string filename: filename of the CSV file specified by the user
        """
        return self.__filename

    def get_csv_headers(self):
        """
        Returns the recorder's headers

        :return string[] headers: The headers separated by a comma in an array
        """
        return self.__headers

    # pylint: disable=unused-argument
    def reset(self, tf_manager):
        """
        Resets the recorder
        """
        self.__reset_since_last_record = True
        if self.__erase_on_reset:
            header = self.__headers
            del self.__values[:]
            self.__values.append(header)

        return self

    def cleanup(self):
        """
        Retrieves the csv data recorded values and cleans up the values buffer

        :return string[] values: A string array containing the properly formatted
        comma separated recorder values
        """
        # expand the array of values which is an array of arrays
        # we have a newline character already appended to the values
        # in the appropriate place to signal when the current line ends
        values = [item for sublist in self.__values for item in sublist]

        # flush the buffer which holds the values
        del self.__values[:]

        return values
