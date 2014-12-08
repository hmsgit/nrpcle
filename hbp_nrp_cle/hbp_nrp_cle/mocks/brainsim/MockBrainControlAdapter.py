'''
MockControlAdapter.py
moduleauthor: Michael.Weber@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import IBrainControlAdapter

__author__ = 'MichaelWeber'


class MockBrainControlAdapter(IBrainControlAdapter):
    """
    Represents a mocked controller object for the neuronal simulator
    """
    #This is a mock, there might be unused parameters
    #pylint: disable=W0613
    def __init__(self):
        """
        Initializes the Mock control adapter
        """
        self.__is_initialized = False
        self.__is_alive = False
        self.__rank = None

    def initialize(self, **params):
        """
        Initializes the neuronal simulator
        :param params: A dictionary of configuration parameters
        """
        self.__is_initialized = True
        self.__is_alive = True
        self.__rank = 0

        print "Setting up MOCK."
        return self.__is_initialized

    def is_alive(self):  # -> bool:
        """
        Gets a status whether the neuronal simulator is still alive
        :return: True if the simulator is alive, otherwise False
        """
        return self.__is_alive

    def run_step(self, dt):  # -> None:
        """
        Runs the neuronal simulator for the given amount of simulated time
        :param dt: the simulated time in milliseconds
        """

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        self.__is_alive = False
