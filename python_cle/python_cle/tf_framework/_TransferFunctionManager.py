"""
This module contains the implementation of a transfer functions manager
"""

__author__ = 'GeorgHinkel'

from python_cle.robotsim.RobotInterface import IRobotCommunicationAdapter
from python_cle.brainsim.BrainInterface import IBrainCommunicationAdapter
from ._Neuron2Robot import Neuron2Robot, MapNeuronParameter
from ._Robot2Neuron import Robot2Neuron, MapRobotParameter
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
        self.__r2n = []
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
        self.__nestAdapter.refresh_buffers(t)

        for _n2r in self.__n2r:
            assert isinstance(_n2r, Neuron2Robot)
            _n2r.run(t)

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation
        :param t:  The simulation time
        """
        self.__robotAdapter.refresh_buffers(t)

        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            _r2n.run(t)

    def initialize(self, name):
        """
        Initializes the transfer Function node with the given name
        :param name: The name for this transfer function node
        """
        if self.__nestAdapter is None:
            raise Exception("No brain simulation adapter has been specified")

        if self.__robotAdapter is None:
            raise Exception("No robot simulation adapter has been specified")

        assert isinstance(self.__nestAdapter, IBrainCommunicationAdapter)
        assert isinstance(self.__robotAdapter, IRobotCommunicationAdapter)

        # Wire transfer functions from neuronal simulation to world simulation
        for _n2r in self.__n2r:
            assert isinstance(_n2r, Neuron2Robot)
            _n2r.replace_params()

            _n2r.topic = self.__robotAdapter.register_publish_topic(
                _n2r.topic)
            for i in range(0, len(_n2r.topics)):
                _n2r.topics[i] = self.__robotAdapter.register_publish_topic(
                    _n2r.topics[i])

            for i in range(1, len(_n2r.neuron_params)):
                param = _n2r.neuron_params[i]
                assert isinstance(param, MapNeuronParameter)
                _n2r.neuron_params[i] = self.__nestAdapter \
                    .register_spike_sink(self.__select_neurons(param.neurons), param.device_type,
                                         **param.config)
                _n2r.__dict__[param.name] = _n2r.neuron_params[i]

        # Wire transfer functions from world simulation to neuronal simulation
        for _r2n in self.__r2n:
            assert isinstance(_r2n, Robot2Neuron)
            _r2n.check_params()

            for i in range(1, len(_r2n.params)):
                param = _r2n.params[i]
                if isinstance(param, MapRobotParameter):
                    _r2n.params[i] = self.__robotAdapter \
                        .register_subscribe_topic(param.topic, **param.config)
                else:
                    assert isinstance(param, MapNeuronParameter)
                    _r2n.params[i] = self.__nestAdapter \
                        .register_spike_source(self.__select_neurons(param.neurons),
                                               param.device_type, **param.config)
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
