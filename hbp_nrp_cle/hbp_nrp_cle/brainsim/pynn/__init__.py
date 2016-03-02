"""
This package contains the brain adapter implementation relying on the PyNN neuronal simulator
abstraction layer.
"""

__author__ = "Sebastian Krach, Georg Hinkel"

from .__simulator import simulator
from hbp_nrp_cle.brainsim.BrainInterface import PopulationInfo


class PyNNPopulationInfo(PopulationInfo):
    """
    The PyNN implementation of populations
    """

    def __init__(self, population, name):
        """
        Creates a new PyNN population based on the given population and the given name

        :param population: The underlying PyNN population
        :param name: The name for the population
        """
        self.__population = population
        self.__name = name

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
        """
        if hasattr(self.__population, "celltype"):
            return type(self.__population.celltype).__name__
        else:
            return "PopulationAssembly"

    @property
    def parameters(self):
        """
        Gets the parameters of a the population as dict
        """
        if hasattr(self.__population, "celltype"):
            return self.__population.celltype.parameters
        else:
            return {}

    @property
    def gids(self):
        """
        Gets the global unique identifiers for this population
        """
        return [long(i) for i in self.__population.all()]
