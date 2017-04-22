# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
This module contains functions supporting monitoring of neurons
"""

__author__ = 'GeorgHinkel'

from cle_ros_msgs.msg import SpikeEvent, SpikeData


def create_spike_recorder_message(t, neuron_count, spikes, monitor_name):
    """
    Creates a monitoring message for the given simulation time and spike counts

    :param t: The simulation time
    :param neuron_count: The amount of neurons monitored in total
    :param spikes: The spikes received
    :param monitor_name: The name of the monitor
    :return:
    """

    if spikes is None or spikes.shape[1] == 0:
        return SpikeEvent(t, 0, [], monitor_name)  # return a SpikeEvent message with zero spikes

    msgs = []
    for spike in spikes:
        msgs.append(SpikeData(int(spike[0]) - 3, spike[1]))  # Neuron ID start at 1.
    return SpikeEvent(t, neuron_count, msgs, monitor_name)
