'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IBrainControlAdapter
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNControlAdapter(IBrainControlAdapter):
    """
    Represents a controller object for the neuronal simulator
    """
    def __init__(self):
        """
        Initializes the PyNN control adapter
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
        self.__rank = sim.setup(**params)

        print "Setting up process %d." % self.__rank

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
        sim.run(dt)

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        self.__is_alive = False
        sim.end()
