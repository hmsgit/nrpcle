from tf_framework.TransferFunctionInterface import ITransferFunctionManager

__author__ = 'GeorgHinkel'

class MockTransferFunctionManager(ITransferFunctionManager):
    """
    Mocks the transfer functions manager
    """

    def __init__(self):
        """
        Initializes a new transfer functions mock
        """
        self.__name = None
        self.__r2nTimes = []
        self.__n2rTimes = []

    def initialize(self, name):
        """
        Initializes the mocked transfer functions
        :param name: The user friendly name of this manager
        """
        self.__name = name

    def run_neuron_to_robot(self, t):
        """
        Runs the transfer function mocks for neuron to robot direction
        :param t: The simulation time of the neuronal simulator
        """
        self.__n2rTimes.append(t)

    def run_robot_to_neuron(self, t):
        """
        Runs the transfer function mocks for the robot to neuron direction
        :param t: The simulation time of the world simulator
        """
        self.__r2nTimes.append(t)

    @property
    def name(self):
        """
        Gets the name of the current transfer function manager mock
        """
        return self.__name

    @property
    def is_initialized(self):
        """
        Gets a value indicating whether the transfer function manager has been initialized
        """
        return self.__name is not None

    @property
    def robot_to_neuron_times(self):
        """
        Gets the times the transfer function manager has been called from robot to neurons
        """
        return self.__r2nTimes

    @property
    def neuron_to_robot_times(self):
        """
        Gets the times the transfer function manager has been called from neuron to robot
        """
        return self.__n2rTimes