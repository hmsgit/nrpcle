"""
This package contains the classes to specify transfer functions and connect them to communication
adapters of both the neuronal simulator and the world simulator
"""


from hbp_nrp_cle.common import UserCodeException


class TFException(UserCodeException):
    """
    Exception class used to return a meaningful message
    to ExD front-end in case the update of TF's user code
    fails.

    :param tf_name: name of the TF updated by the user.
    :param message: message that needs to be forwarded to the front-end.
    """

    def __init__(self, tf_name, message, error_type):
        super(TFException, self).__init__(message, error_type)
        self.tf_name = tf_name

    def __str__(self):
        return "{0}: {1} ({2})".format(self.tf_name, repr(self.message), self.error_type)


class TFLoadingException(TFException):
    """
    Exception class used to return a meaningful message
    to ExD front-end in case the loading of a TF with updated user code
    fails.

    :param tf_name: name of the TF updated by the user.
    :param message: message that needs to be forwarded to the front-end.
    """

    def __init__(self, tf_name, message):
        super(TFLoadingException, self).__init__(tf_name, message, 'TF Loading Exception')
from . import config

from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, IPopulationRate, ISpikeRecorder

import logging
logger = logging.getLogger(__name__)

# alias _Facade module needed by set_transfer_function
import sys
nrp = sys.modules[__name__]


# CLE restricted python environment
# TODO(Luc): create a module for it
from RestrictedPython.PrintCollector import PrintCollector
from operator import getitem

# pylint: disable=unused-import
from ._Neuron2Robot import Neuron2Robot, MapSpikeSink, MapSpikeSource
from ._Robot2Neuron import Robot2Neuron, MapRobotPublisher, \
    MapRobotSubscriber
from hbp_nrp_cle.tf_framework._TransferFunction import TransferFunction
from hbp_nrp_cle.tf_framework._CSVRecorder import MapCSVRecorder, CSVRecorder
from hbp_nrp_cle.tf_framework._NeuronMonitor import NeuronMonitor
from hbp_nrp_cle.tf_framework._GlobalData \
    import MapVariable, MapRetina, PyRetinaWrapper, GLOBAL, TRANSFER_FUNCTION_LOCAL
from hbp_nrp_cle.tf_framework._CleanableTransferFunctionParameter \
    import ICleanableTransferFunctionParameter
from . import _TransferFunctionManager
from . import _PropertyPath
from . import _NeuronSelectors
from ._TransferFunctionInterface import ITransferFunctionManager
from hbp_nrp_cle.robotsim.RobotInterface import Topic, IRobotCommunicationAdapter
import std_msgs.msg
import cle_ros_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import gazebo_msgs.msg
import hbp_nrp_cle.tf_framework.monitoring
import hbp_nrp_cle.tf_framework.tf_lib
from hbp_nrp_cle.logs import clientLogger
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Float32, Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np

__author__ = 'Georg Hinkel'

_getattr_ = getattr
_getitem_ = getitem
_getiter_ = iter
_print_ = PrintCollector


def _cle_write_guard():
    """
    Defines the write guard for execution of user code in restricted mode.
    """

    def guard(ob):
        """
        No guard at all
        """
        return ob

    return guard


cle_write_guard = _cle_write_guard()
_write_ = cle_write_guard

leaky_integrator_alpha = ILeakyIntegratorAlpha
leaky_integrator_exp = ILeakyIntegratorExp
fixed_frequency = IFixedSpikeGenerator
poisson = IPoissonSpikeGenerator
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
    config.csv_recorders = []


def get_transfer_functions():
    """
    Get all the transfer functions

    :return: All the transfer functions (R2N, N2R).
    """
    return config.active_node.n2r + config.active_node.r2n


def get_transfer_function(name):
    """
    Get the transfer function with the given name

    :param name: The name of the transfer function
    :return: The transfer function with the given name
    """

    return next((tf for tf in get_transfer_functions() if tf.name == name), None)


def get_brain_source():
    """
    Get the source of the brain (if loaded from a python file). Otherwise, returns
    None.

    :return: The source of the brain model
    """
    return config.brain_source


def get_brain_populations():
    """
    Get the brain populations as a dictionnary
    If the brain model is not loaded,
    the function returns None.

    :return: A dictionnary containing the brain populations
    The dictionnary keys are population names
    and its values are one of the following types:
    list, or a 'slice' dictionnary of the following form
    {'from': 1, 'to': 10, 'step': 1}.
    """
    return config.brain_populations


def dump_csv_recorder_to_files():
    """
    Find out all CSV recorders and dump their values to CSV files.

    :return: an array of pairs containing the filename wanted by the user and a temporary
    filepath to a file containing the values.
    """
    result = []
    for tf in get_transfer_functions():
        print tf.params
        for i in range(1, len(tf.params)):
            if isinstance(tf.params[i], CSVRecorder):
                name, temporary_path = tf.params[i].dump_to_file()
                result.append([name, temporary_path])
    return result


def clean_csv_recorders_files():
    """
    Clean out all CSV recorders generated files.
    """
    for tf in get_transfer_functions():
        print tf.params
        for i in range(1, len(tf.params)):
            if isinstance(tf.params[i], CSVRecorder):
                tf.params[i].cleanup()


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
            if tf.params[i] in config.active_node.brain_adapter.detector_devices:
                config.active_node.brain_adapter.detector_devices.remove(tf.params[i])
            if isinstance(tf.params[i], ICleanableTransferFunctionParameter):
                tf.params[i].cleanup()
        config.active_node.n2r.remove(tf)
    elif tf in config.active_node.r2n:
        config.active_node.r2n.remove(tf)
        for i in range(1, len(tf.params)):
            if tf.params[i] in config.active_node.brain_adapter.generator_devices:
                config.active_node.brain_adapter.generator_devices.remove(tf.params[i])
            if isinstance(tf.params[i], ICleanableTransferFunctionParameter):
                tf.params[i].cleanup()
    else:
        result = False
    return result


def set_transfer_function(new_source, new_code, new_name):
    """
    Apply transfer function changes made by a client

    :param new_source: Transfer function's updated source
    :param new_code: Compiled code of the updated source
    :param new_name: Transfer function's updated name
    """

    # pylint: disable=broad-except
    try:
        # pylint: disable=exec-used
        exec new_code
        tf = get_transfer_function(new_name)
        if not isinstance(tf, TransferFunction):
            raise Exception("Transfer function has no decorator specifying its type")
        config.active_node.initialize_tf(tf)
    except Exception as e:
        logger.error("Error while loading new transfer function")
        logger.error(e)
        delete_transfer_function(new_name) # prevents runtime error
        raise TFLoadingException(new_name, str(e))

    # we set the new source in an attribute because inspect.getsource won't work after exec
    # indeed inspect.getsource is based on a source file object
    # see findsource in http://www.opensource.apple.com/source/python/python-3/python/Lib/inspect.py
    tf.source = new_source

start_new_tf_manager()
