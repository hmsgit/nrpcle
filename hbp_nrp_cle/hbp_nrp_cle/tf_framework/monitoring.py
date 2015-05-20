"""
This module contains functions supporting monitoring of neurons
"""

__author__ = 'GeorgHinkel'

from cle_ros_msgs.msg import SpikeEvent, SpikeData


def create_spike_recorder_message(t, neuronCount, spikes, monitor_name):
    """
    Creates a monitoring message for the given simulation time and spike counts
    :param t: The simulation time
    :param neuronCount: The amount of neurons monitored in total
    :param spikes: The spikes received
    :param monitor_name: The name of the monitor
    :return:
    """
    if spikes is None or spikes.shape[1] == 0:
        return None

    msgs = []
    for spike in spikes:
        msgs.append(SpikeData(int(spike[0]) - 1, spike[1])) # Neuron ID start at 1.
    return SpikeEvent(t, neuronCount, msgs, monitor_name)
