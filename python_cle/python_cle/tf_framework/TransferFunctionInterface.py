__author__ = 'GeorgHinkel'


class ITransferFunctionManager(object):  # pragma: no cover
    """
    Represents the interface of a transfer functions node
    """

    def run_neuron_to_robot(self, t):  # -> None:
        """
        Runs the transfer functions from the neuronal simulator towards the robot
        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation
        :param t:  The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self, name):  # -> None:
        """
        Initializes the transfer Function node with the given name
        :param name: The name for this transfer function node
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
