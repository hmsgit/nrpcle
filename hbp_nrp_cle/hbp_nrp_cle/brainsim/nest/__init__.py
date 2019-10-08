"""
This package contains the brain adapter implementation relying on the NEST neuronal simulator
abstraction layer.
"""

from hbp_nrp_cle.brainsim.BrainInterface import PopulationInfo

import nest


class NestPopulationInfo(PopulationInfo):
    """
    The NEST implementation of populations
    """

    def __init__(self, population, name, parameters):
        """
        Creates a new NEST population based on the given population and the given name

        :param population: The underlying NEST population
        :param name: The name for the population
        :param parameters: The population parameters
        """
        self.__population = population
        self.__name = name
        self.__parameters = parameters

    @property
    def name(self):
        """
        Gets the population name
        """
        return self.__name

    @property
    def celltype(self):
        """
        Gets the celltype of the population
        Population is assumed homogeneous, i.e. the elements are of the same type
        """
        return str(nest.GetStatus(self.__population, 'model')[0])

    @property
    def parameters(self):
        """
        Gets the parameters of a the population as dict
        """
        return self.__parameters

    @property
    def gids(self):
        """
        Gets the global unique identifiers for this population
        """
        return self.__population

    @property
    def indices(self):
        """
        Gets the population info indices
        """
        return range(len(self.__population))
