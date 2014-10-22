"""
This module contains the publicly visible interface for the transfer functions
framework that allows the neuroscience user to conveniently specify the
transfer functions
"""

from . import config

from python_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, \
    ISpikeDetector, IDCSource, IACSource, INCSource, IPopulationRate

# pylint: disable=W0611

from ._Neuron2Robot import Neuron2Robot, MapNeuronParameter
from ._Robot2Neuron import Robot2Neuron, MapRobotParameter
from . import _TransferFunctionManager, _PropertyPath

__author__ = 'GeorgHinkel'

leaky_integrator_alpha = ILeakyIntegratorAlpha
leaky_integrator_exp = ILeakyIntegratorExp
fixed_frequency = IFixedSpikeGenerator
poisson = IPoissonSpikeGenerator
detector = ISpikeDetector
dc_source = IDCSource
ac_source = IACSource
nc_source = INCSource
population_rate = IPopulationRate

brain = _PropertyPath.PropertyPath()


def initialize(name):  # -> None:
    """
    Initializes and starts the TF node
    :param name: The name of the TF node
    """
    config.active_node.initialize(name)


def send_robot(topic, value):  # -> None:
    """
    Send data to the given robot topic
    :param topic: The robot topic
    :param value: The values sent to the robot
    """
    print("Sending ", value, " to robot ", topic)


def set_nest_adapter(nest_adapter):  # -> None:
    """
    Sets the brainsim adapter. Must be executed before tf node initialization
    :param nest_adapter: The brainsim adapter
    """
    config.active_node.brain_adapter = nest_adapter


def set_robot_adapter(robot_adapter):  # -> None:
    """
    Sets the robot adapter. Must be run before tf node initialization
    :param robot_adapter: The robot adapter
    """
    config.active_node.robot_adapter = robot_adapter


def start_new_tf_manager():
    """
    Start a new transfer function manager
    """
    config.active_node = _TransferFunctionManager.TransferFunctionManager()
