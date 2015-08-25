"""
This module contains the publicly visible interface for the transfer functions
framework that allows the neuroscience user to conveniently specify the
transfer functions
"""

from . import config

import textwrap
import re
import collections
TFCompileOutput = collections.namedtuple(
    'TFCompileOutput',
    ['compile_error', 'new_source', 'new_code', 'new_name']
)

from RestrictedPython import compile_restricted
from RestrictedPython.PrintCollector import PrintCollector
from operator import getitem
_getattr_ = getattr
_getitem_ = getitem
_print_ = PrintCollector

from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, \
    ISpikeDetector, IDCSource, IACSource, INCSource, IPopulationRate, ISpikeRecorder

import logging
logger = logging.getLogger(__name__)

# alias _Facade module needed by set_transfer_function
import sys
nrp = sys.modules[__name__]

# The following modules are needed for transfer function code's execution
# pylint: disable=unused-import
from ._Neuron2Robot import Neuron2Robot, MapSpikeSink, MapSpikeSource
from ._Robot2Neuron import Robot2Neuron, MapRobotPublisher, MapRobotSubscriber
from . import _TransferFunctionManager, _PropertyPath, _NeuronSelectors
from ._TransferFunctionInterface import ITransferFunctionManager
from hbp_nrp_cle.robotsim.RobotInterface import Topic, IRobotCommunicationAdapter
import cle_ros_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from hbp_nrp_cle.tf_framework import monitoring
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Float32, Int32, String

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
spike_recorder = ISpikeRecorder


brain = _PropertyPath.PropertyPath()


def map_neurons(neuron_range, mapping):
    """
    Maps the given range to neurons using the provided mapping
    :param neuron_range: A range that can be iterated
    :param mapping: A mapping function or lambda
    """
    return _NeuronSelectors.MapNeuronSelector(neuron_range, mapping)


def chain_neurons(*neuron_selectors):
    """
    Chains the given neuron selectors
    :param neuron_selectors: The neuron selectors
    """
    return _NeuronSelectors.ChainNeuronSelector(neuron_selectors)


def initialize(name):  # -> None:
    """
    Initializes and starts the TF node

    :param name: The name of the TF node
    """
    config.active_node.initialize(name)


def set_nest_adapter(nest_adapter):  # -> None:
    """
    Sets the brainsim adapter.

    :param nest_adapter: The brainsim adapter

    .. WARNING:: Must be executed before tf node initialization
    """
    config.active_node.brain_adapter = nest_adapter


def set_robot_adapter(robot_adapter):  # -> None:
    """
    Sets the robot adapter.

    :param robot_adapter: The robot adapter

    .. WARNING:: Must be executed before tf node initialization
    """
    config.active_node.robot_adapter = robot_adapter


def start_new_tf_manager():
    """
    Start a new transfer function manager
    """
    config.active_node = _TransferFunctionManager.TransferFunctionManager()


def get_transfer_functions():
    """
    Get all the transfer functions

    :return: All the transfer functions (R2N and N2R).
    """
    return config.active_node.n2r + config.active_node.r2n


def get_transfer_function(name):
    """
    Get the transfer function with the given name

    :param name: The name of the transfer function
    :return: The transfer function with the given name
    """

    return next((tf for tf in get_transfer_functions() if tf.name == name), None)


def delete_transfer_function(name):
    """
    Delete a transfer function. If the transfer function does not exist,
    nothing will happen.

    :param name: The name of the transfer function
    :return: True if the transfer function is correctly deleted. False if the transfer function
             does not exist.
    """
    result = True
    tf = get_transfer_function(name)
    if tf in config.active_node.n2r:
        for i in range(1, len(tf.params)):
            if (tf.params[i] in config.active_node.brain_adapter.detector_devices):
                config.active_node.brain_adapter.detector_devices.remove(tf.params[i])
        config.active_node.n2r.remove(tf)
    elif tf in config.active_node.r2n:
        config.active_node.r2n.remove(tf)
        for i in range(1, len(tf.params)):
            if (tf.params[i] in config.active_node.brain_adapter.generator_devices):
                config.active_node.brain_adapter.generator_devices.remove(tf.params[i])
    else:
        result = False
    return result


def compile_transfer_function(new_transfer_function_source):
    """
    Compile synchronously transfer function's source in restricted mode

    :param new_transfer_function_source: Sources of the transfer function
    :param original_name: Name of the transfer function to replace (None for a new tf)
    :return: TFCompileOutput("", new_source, new_code, new_name)
             if the compilation is successfull,
             TFCompileOutput(compile_error, new_source, None, new_name)
             if the compilation fails but the new function has a well-defined definition name
             TFCompileOutput(compile_error, new_source, None, None)
             if the new function has no well-defined definition name
    """

    # Update transfer function's source code
    new_source = textwrap.dedent(new_transfer_function_source)

    # Compile transfer function's new code in restricted mode
    logger.debug(
        "About to compile transfer function with the following python code: \n"
        + repr(new_source)
    )
    m = re.findall(r"def\s+(\w+)\s*\(", new_source)
    compile_error = ""
    if (len(m) != 1):
        compile_error = \
            "Transfer function contains either no or multiple definition names. \
            Compilation aborted."
        return TFCompileOutput(compile_error, new_source, None, None)

    new_name = m[0]
    new_code = None
    try:
        new_code = compile_restricted(new_source, '<string>', 'exec')
    except SyntaxError as e:
        logger.error("Syntax Error while compiling new transfer function in restricted mode")
        logger.error(e)
        compile_error = e
    return TFCompileOutput(compile_error, new_source, new_code, new_name)


def set_transfer_function(new_source, new_code, new_name):
    """
    Apply transfer function changes made by a client

    :param new_source: Transfer function's updated source
    :param new_code: Compiled code of the updated source
    :param new_name: Transfer function's updated name
    :return: True if the new source code is successfully loaded, False otherwise
    """

    # pylint: disable=broad-except
    result = True
    try:
        # pylint: disable=exec-used
        exec(new_code)
        tf = get_transfer_function(new_name)
        if isinstance(tf, Neuron2Robot):
            config.active_node.initialize_n2r_tf(tf)
        elif isinstance(tf, Robot2Neuron):
            config.active_node.initialize_r2n_tf(tf)
        else:
            logger.error("Transfer function has no decorator r2n or n2r")
            result = False
    except Exception as e:
        logger.error("Error while loading new transfer function")
        logger.error(e)
        result = False

    # we set the new source in an attribute because inspect.getsource won't work after exec
    # indeed inspect.getsource is based on a source file object
    # see findsource in http://www.opensource.apple.com/source/python/python-3/python/Lib/inspect.py
    if result:
        tf.set_source(new_source)

    return result
