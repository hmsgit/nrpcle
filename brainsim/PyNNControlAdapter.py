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
    def __init__(self, **kwargs):
        """
        Initializes the neuronal simulator
        :param kwargs: A dictionary of configuration parameters
        """
        self.__is_alive = True
        rank = sim.setup(**kwargs)

        print "Setting up process %d." % rank

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
