"""
This module contains the implementation of a transfer functions manager
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter
from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter
from ._Neuron2Robot import Neuron2Robot, MapSpikeSink
from ._Robot2Neuron import Robot2Neuron
from ._TransferFunctionInterface import ITransferFunctionManager
from ._PropertyPath import PropertyPath
from . import config
import logging
import time

logger = logging.getLogger(__name__)


class TransferFunctionManager(ITransferFunctionManager):
    """
    Represents a transfer functions node
    """

    def __init__(self):  # -> None:
        """
        Creates a new transfer functions node
        """
        self.__n2r = []
        self.__n2r_old = []
        self.__r2n = []
        self.__r2n_old = []
        self.__robotAdapter = None
        self.__nestAdapter = None
        self.__initialized = False

    @property
    def n2r(self):  # -> list:
        """
        Gets a list of transfer functions from the neuronal simulator to the world simulation
        """
        return self.__n2r

    @property
    def r2n(self):  # -> list:
        """
        Gets a list of transfer functions from the world simulator to the neuronal simulation
        """
        return self.__r2n

    def run_neuron_to_robot(self, t):  # -> None:
        """
        Runs the transfer functions from the neuronal simulator towards the robot

        :param t: The simulation time
        """
        for _n2r in self.__n2r:
            assert isinstance(_n2r, Neuron2Robot)
            start = time.time()
            _n2r.run(t)
            _n2r.elapsed_time += time.time() - start

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation

        :param t:  The simulation time
        """
        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            start = time.time()
            _r2n.run(t)
            _r2n.elapsed_time += time.time() - start

    def initialize_n2r_tf(self, tf):
        """
        Initializes the given neuron-to-robot transfer function

        :param name: The transfer function
        """
        # Wire transfer function from neuronal simulation to world simulation
        assert isinstance(tf, Neuron2Robot)
        logger.info("Initialize transfer function " + repr(tf))
        tf.replace_params()
        tf.elapsed_time = 0.0

        if tf.topic is not None:
            tf.topic = self.__robotAdapter.register_publish_topic(tf.topic)

        for i in range(1, len(tf.params)):
            param = tf.params[i]
            assert isinstance(param, MapSpikeSink)
            # TODO: removes former adapters (pyNN neuron Populations)
            # if the transfer function code is updated
            tf.params[i] = tf.params[i].create_adapter(self)
            tf.__dict__[param.name] = tf.params[i]

    def initialize_r2n_tf(self, tf):
        """
        Initializes the given robot-to-neuron transfer function

        :param name: The transfer function
        """
        # Wire transfer function from world simulation to neuronal simulation
        assert isinstance(tf, Robot2Neuron)
        logger.info("Initialize transfer function " + repr(tf))
        tf.check_params()
        tf.elapsed_time = 0.0

        for i in range(1, len(tf.params)):
            param = tf.params[i]
            tf.params[i] = tf.params[i].create_adapter(self)
            tf.__dict__[param.name] = tf.params[i]

    def initialize(self, name):
        """
        Initializes the transfer Function node with the given name

        :param name: The name for this transfer function node
        """
        logger.info("Initialize transfer functions node " + name)
        if self.__nestAdapter is None:
            raise Exception("No brain simulation adapter has been specified")

        if self.__robotAdapter is None:
            raise Exception("No robot simulation adapter has been specified")

        assert isinstance(self.__nestAdapter, IBrainCommunicationAdapter)
        assert isinstance(self.__robotAdapter, IRobotCommunicationAdapter)

        # Wire transfer functions from neuronal simulation to world simulation
        for _n2r in self.__n2r:
            self.initialize_n2r_tf(_n2r)

        # Wire transfer functions from world simulation to neuronal simulation
        for _r2n in self.__r2n:
            self.initialize_r2n_tf(_r2n)

        # Initialize dependencies
        self.__nestAdapter.initialize()
        self.__robotAdapter.initialize(name)

    @staticmethod
    def __select_neurons(neurons):
        """
        Selects the neurons represented by the given property path

        :param neurons:
        """
        if isinstance(neurons, PropertyPath):
            return neurons.select(config.brain_root)
        if isinstance(neurons, list):
            for i in range(0, len(neurons)):
                neurons[i] = TransferFunctionManager.__select_neurons(neurons[i])
        return neurons

    @property
    def robot_adapter(self):  # -> IRobotCommunicationAdapter:
        """
        Gets or sets the adapter to the world simulation
        """
        return self.__robotAdapter

    @robot_adapter.setter
    def robot_adapter(self, robot_adapter):  # -> None:
        """
        Sets the robot adapter

        :param robot_adapter: The new robot adapter
        """
        if self.__initialized:
            raise Exception("Cannot exchange robot adapter after node has been initialized!")
        else:
            assert isinstance(robot_adapter, IRobotCommunicationAdapter)
            self.__robotAdapter = robot_adapter

    @property
    def brain_adapter(self):  # -> IBrainCommunicationAdapter:
        """
        Gets or sets the adapter to the brain simulation
        """
        return self.__nestAdapter

    @brain_adapter.setter
    def brain_adapter(self, nest_adapter):  # -> None:
        """
        Sets the nest adapter

        :param nest_adapter: The new brain simulation adapter
        """
        if self.__initialized:
            raise Exception("Cannot exchange brainsim adapter after node has been initialized!")
        else:
            assert isinstance(nest_adapter, IBrainCommunicationAdapter)
            self.__nestAdapter = nest_adapter

    def transfer_functions(self):
        """
        Gets a list of transfer functions managed by this instance

        :return: A list of transfer functions
        """
        return self.__n2r + self.__r2n

    def __reset_neuron2robot(self, _n2r):
        """
        Resets the given neuron2robot transfer function
        """
        assert isinstance(_n2r, Neuron2Robot)
        _n2r.elapsed_time = 0.0
        _n2r.replace_params()
        if _n2r.topic is not None:
            _n2r.topic = _n2r.topic.reset(self)
        for i in range(1, len(_n2r.params)):
            param = _n2r.params[i]
            reset_value = param.reset(self)
            if param is not reset_value:
                _n2r.params[i] = reset_value
                for k in _n2r.__dict__:
                    if _n2r.__dict__[k] is param:
                        _n2r.__dict__[k] = reset_value

    def __reset_robot2neuron(self, _r2n):
        """
        Resets the given robot2neuron transfer function
        """
        assert isinstance(_r2n, Robot2Neuron)
        _r2n.check_params()
        _r2n.elapsed_time = 0.0
        for i in range(1, len(_r2n.params)):
            param = _r2n.params[i]
            reset_value = param.reset(self)
            if param is not reset_value:
                _r2n.params[i] = reset_value
                for k in _r2n.__dict__:
                    if k in _r2n.__dict__:
                        _r2n.__dict__[k] = reset_value

    def reset(self):  # -> None:
        """
        Resets the transfer functions
        """
        logger.info("Resetting transfer functions")
        if self.__nestAdapter is None:
            raise Exception("No brain simulation adapter has been specified")

        if self.__robotAdapter is None:
            raise Exception("No robot simulation adapter has been specified")

        assert isinstance(self.__nestAdapter, IBrainCommunicationAdapter)
        assert isinstance(self.__robotAdapter, IRobotCommunicationAdapter)

        # Wire transfer functions from neuronal simulation to world simulation
        for _n2r in self.__n2r:
            self.__reset_neuron2robot(_n2r)

        for _r2n in self.__r2n:
            self.__reset_robot2neuron(_r2n)

        # Initialize dependencies
        self.__nestAdapter.initialize()
