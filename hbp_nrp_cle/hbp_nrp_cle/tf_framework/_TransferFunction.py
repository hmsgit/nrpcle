"""
Defines the base class for a transfer function
"""

__author__ = 'GeorgHinkel'


class TransferFunction(object):
    """
    Represents the base class for a transfer function
    """

    def __init__(self):
        self._params = []

    @property
    def params(self):
        """
        Gets the simulation adapters for this transfer function, either robot subscriber, publisher
        or neuronal simulator device adapters

        :return: A list of adapters
        """
        return self._params
