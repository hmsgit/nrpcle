"""
This module contains the implementation of a transfer functions manager
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter
from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter
from ._Neuron2Robot import Neuron2Robot
from ._Robot2Neuron import Robot2Neuron
from ._TransferFunctionInterface import ITransferFunctionManager
from ._PropertyPath import PropertyPath
from . import config


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
            _n2r.run(t)

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation
        :param t:  The simulation time
        """
        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            _r2n.run(t)

    def initialize(self, name):
        """
        Initializes the transfer Function node with the given name
        :param name: The name for this transfer function node
        """
        print("Initialize transfer functions node ", name)
        if self.__nestAdapter is None:
            raise Exception("No brain simulation adapter has been specified")

        if self.__robotAdapter is None:
            raise Exception("No robot simulation adapter has been specified")

        assert isinstance(self.__nestAdapter, IBrainCommunicationAdapter)
        assert isinstance(self.__robotAdapter, IRobotCommunicationAdapter)

        # Wire transfer functions from neuronal simulation to world simulation
        for _n2r in self.__n2r:
            assert isinstance(_n2r, Neuron2Robot)
            print("Initialize transfer function ", repr(_n2r))
            _n2r.replace_params()

            if _n2r.topic is not None:
                _n2r.topic = self.__robotAdapter.register_publish_topic(_n2r.topic)

            for i in range(1, len(_n2r.params)):
                param = _n2r.params[i]
                _n2r.params[i] = _n2r.params[i].create_adapter(self)
                _n2r.__dict__[param.name] = _n2r.params[i]

        # Wire transfer functions from world simulation to neuronal simulation
        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            print("Initialize transfer function ", repr(_r2n))
            _r2n.check_params()

            for i in range(1, len(_r2n.params)):
                param = _r2n.params[i]
                _r2n.params[i] = _r2n.params[i].create_adapter(self)
                _r2n.__dict__[param.name] = _r2n.params[i]

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

    def __reset_neuron2robot(self, _n2r):
        """
        Resets the given neuron2robot transfer function
        """
        assert isinstance(_n2r, Neuron2Robot)
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
        print("Resetting transfer functions")
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
