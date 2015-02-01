'''
PyNNControlAdapter.py
moduleauthor: probst@fzi.de
'''

from .BrainInterface import IBrainControlAdapter
import pyNN.nest as sim
from . import BrainLoader
import logging

logger = logging.getLogger(__name__)

__author__ = 'DimitriProbst'


class PyNNControlAdapter(IBrainControlAdapter):
    """
    Represents a controller object for the neuronal simulator
    """

    def __init__(self, network_file='', **populations):
        """
        Initializes the PyNN control adapter
        :param network_file: The path to the .5h file containing the network
        :param sensors: list of sensor units IDs
        :param actors: list of actor units IDs
        """
        self.__is_initialized = False
        self.__is_alive = False
        self.__rank = None

        self.__network_file = network_file
        self.__populations = populations

    def initialize(self, **params):
        """
        Initializes the neuronal simulator
        :param params: A dictionary of configuration parameters
        :return: True if the simulator is initialized, otherwise False
        """
        if not self.__is_initialized:
            timestep = params.get('timestep', 0.1)
            min_delay = params.get('min_delay', 0.1)
            max_delay = params.get('max_delay', 20.0)
            threads = params.get('threads', 1)
            rng_seeds = params.get('rng_seeds', [1234])
            self.__rank = sim.setup(timestep=timestep, min_delay=min_delay,
                                    max_delay=max_delay, threads=threads,
                                    rng_seeds=rng_seeds)
            if not self.__network_file == '':
                BrainLoader.load_h5_network(self.__network_file,
                                            self.__populations)
            self.__is_initialized = True
            self.__is_alive = True
            logger.info("neuronal simulator initialized")
        else:
            logger.warn("trying to initialize an already initialized controller")
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
        sim.run(dt)

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        self.__is_alive = False
        sim.end()
        logger.info("neuronal simulator ended")

    def reset(self):  # -> None:
        """
        Resets the neuronal simulator
        """
        self.__rank = sim.setup(timestep=sim.get_time_step(),
                                min_delay=sim.get_min_delay(),
                                max_delay=sim.get_max_delay(),
                                threads=1,
                                rng_seeds=[1234])
        if not self.__network_file == '':
            BrainLoader.load_h5_network(self.__network_file,
                                        self.__populations)
        logger.info("neuronal simulator reset")
