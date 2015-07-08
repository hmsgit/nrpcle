"""
Defines the base class for a transfer function
"""

import inspect
import textwrap
import logging

logger = logging.getLogger(__name__)

__author__ = 'GeorgHinkel'


class TransferFunction(object):
    """
    Represents the base class for a transfer function
    """

    def __init__(self):
        self._params = []
        self._func = None

    @property
    def params(self):
        """
        Gets the simulation adapters for this transfer function, either robot subscriber, publisher
        or neuronal simulator device adapters

        :return: A list of adapters
        """
        return self._params

    def get_source(self):
        """
        Gets the source code of this transfer function.

        :return: A string containing the transfer function source code correctly indented.

        .. WARNING:: The source code is read from the generated file based on the template.
        If someone patches the transfer function, the patched code will not be returned.
        (See python inspect module documentation)
        """
        source = "# Transfer function not loaded properly."
        if (self._func):
            try:
                source = inspect.getsource(self._func)
            except IOError as e:
                error_msg = "The transfer function source code cannot be retrieved."
                logger.error(error_msg)
                logger.error(e)
                source = "# " + error_msg

        return textwrap.dedent(source)
