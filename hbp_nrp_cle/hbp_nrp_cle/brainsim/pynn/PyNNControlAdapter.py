'''
PyNNControlAdapter.py
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim import IBrainControlAdapter
from hbp_nrp_cle.brainsim.pynn import PyNNBrainLoader as BrainLoader
from hbp_nrp_cle.brainsim.pynn import simulator as sim
import logging
from os import path
import copy

logger = logging.getLogger(__name__)

__author__ = 'DimitriProbst', 'DanielPeppicelli', 'LucGuyot'


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

    def load_brain(self, network_file, populations):
        """
        Loads the neuronal network contained in the given file

        :param network_file: The path to the neuronal network file
        :param populations: The populations to create
        """
        self.__is_alive = True
        import hbp_nrp_cle.tf_framework.config as tf_config
        tf_config.brain_populations = self.populations_using_json_slice(populations)
        extension = path.splitext(network_file)[1]
        if extension == ".py":
            self.__load_python_brain(
                network_file,
                self.populations_using_python_slice(populations)
            )
        elif extension == ".h5":
            self.__load_h5_brain(
                network_file,
                self.populations_using_python_slice(populations)
            )
        else:
            msg = "Neuronal network format {0} not supported".format(extension)
            raise Exception(msg)

    def __load_h5_brain(self, network_file, populations):
        """
        Loads the brain model in the given h5 file

        :param network_file: The path to the .5h file containing the network
        :param populations: A named list of populations to create
        """
        if not self.__is_initialized:
            self.initialize()
        BrainLoader.load_h5_network(network_file, populations)

    def __load_python_brain(self, network_file, populations):
        """
        Loads the brain model specified in the given Python script

        :param network_file: The Python file containing the network
        :param populations: A named list of populations to create
        """
        if not self.__is_initialized:
            self.initialize()
        BrainLoader.load_py_network(network_file, populations)

        logger.info("Saving brain source")
        import hbp_nrp_cle.tf_framework.config as tf_config
        with open(network_file) as source:
            tf_config.brain_source = source.read()

    def initialize(self, **params):
        """
        Initializes the neuronal simulator

        :param timestep: The timestep used for the neuronal simulation
        :param min_delay: The minimum delay
        :param max_delay: The maximum delay
        :param threads: The amount of threads that should be used to run the simulation
        :param rng_seeds: The rng seeds for the simulation
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
            self.__is_initialized = True
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
        logger.info("neuronal simulator reset")

    @staticmethod
    def populations_using_json_slice(populations):
        """
        Turn populations defined as python slices into python dicts
        to allow straightforward translation in to json.

        :param populations: a dictionary whose values are either
        integers (indices), python lists or slices.
        Slices can be of two types, either python slices,
        or dictionnaries of the form {'from': 1, 'to': 10, 'step': 2}
        :return: A dictionary where python slices have been replaced
        by dictionaries referred to as 'json slices'.
        """
        result = copy.deepcopy(populations)
        for key, value in populations.iteritems():
            if (isinstance(value, slice)):
                p = {'from': value.start, 'to': value.stop, 'step': value.step}
                result[key] = p
        return result

    @staticmethod
    def populations_using_python_slice(populations):
        """
        Turn slices defined as python dicts
        into python slices.
        Populations with different types as dict are left unchanged.

        :param populations: a dictionary whose values are either
        integers (indices), python lists or slices.
        Slices can be of two types, either python slices,
        or dictionnaries of the form {'from': 1, 'to': 10, 'step': 2}
        :return: A dictionary where 'json slices' (plain python dicts) have been replaced
        by python slices.
        """
        result = copy.deepcopy(populations)
        for key, value in populations.iteritems():
            if (isinstance(value, dict) and 'from' in value and 'to' in value):
                step = value.get('step')
                result[key] = slice(value['from'], value['to'], step)
        return result
