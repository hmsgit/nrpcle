from . import config

from brainsim.BrainInterface import IFixedFrequencySpikeGenerator, \
    INeuronVoltmeter, IPoissonSpikeGenerator, IPatternSpikeGenerator, ISpikeRecorder

from .Neuron2Robot import Neuron2Robot, MapNeuronParameter
from .Robot2Neuron import Robot2Neuron, MapRobotParameter

__author__ = 'GeorgHinkel'

voltmeter = INeuronVoltmeter
fixed_frequency = IFixedFrequencySpikeGenerator
poisson = IPoissonSpikeGenerator
pattern = IPatternSpikeGenerator
recorder = ISpikeRecorder


def initialize(name):
    """
    Initializes and starts the TF node
    :param name: The name of the TF node
    """
    config.active_node.initialize(name)


def send_robot(topic, value):
    """
    Send data to the given robot topic
    :param topic: The robot topic
    :param value: The values sent to the robot
    """
    print("Sending ", value, " to robot ", topic)


def set_nest_adapter(nest_adapter):
    """
    Sets the brainsim adapter. Must be executed before tf node initialization
    :param nest_adapter: The brainsim adapter
    """
    config.active_node.nest_adapter = nest_adapter


def set_robot_adapter(robot_adapter):
    """
    Sets the robot adapter. Must be run before tf node initialization
    :param robot_adapter: The robot adapter
    """
    config.active_node.robot_adapter = robot_adapter
