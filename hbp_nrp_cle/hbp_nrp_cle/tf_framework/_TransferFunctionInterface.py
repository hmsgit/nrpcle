"""
This module represents the interface for a transfer functions manager
"""

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

    def initialize_tf(self, tf):
        """
        Initializes the given transfer function

        This method is used if a transfer function is replaced after initialization of the tf
        manager

        :param name: The transfer function
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self):  # -> None:
        """
        Resets the transfer functions
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def transfer_functions(self):
        """
        Gets a list of transfer functions managed by this instance

        :return: A list of transfer functions
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
