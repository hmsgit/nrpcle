"""
This module contains common definitions for mapping specifications
"""

__author__ = 'georghinkel'

from ._TransferFunction import TransferFunction


class ParameterMappingSpecification(object):
    """
    Defines a parameter mapping for a transfer function
    """

    def __init__(self, name):
        self.__name = name

    def __call__(self, transfer_function):  # -> object:
        """
        Applies the parameter mapping to the given transfer function
        """
        if isinstance(transfer_function, TransferFunction):
            parameters = transfer_function.params
            for i in range(0, len(parameters)):
                if parameters[i] == self.name:
                    parameters[i] = self
                    return transfer_function
        else:
            raise Exception("Can only map parameters for transfer functions")
        raise Exception(
            "Could not map parameter as no parameter with the name " + self.name + " exists")

    def create_adapter(self, tf_manager):
        """
        Creates the adapter for this mapping operator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__name

    # pylint: disable=no-self-use
    @property
    def is_brain_connection(self):
        """
        Returns whether the the parameter is connected to the neuronal network

        :return: True, if the parameter is mapped to the neuronal network, otherwise False
        """
        return False

    # pylint: disable=no-self-use
    @property
    def is_robot_connection(self):
        """
        Returns whether the parameter is connected to the simulated robot

        :return: True, if the parameter is mapped to the simulated robot, otherwise False
        """
        return False
