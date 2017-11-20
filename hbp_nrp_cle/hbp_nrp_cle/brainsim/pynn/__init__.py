"""
This package contains the brain adapter implementation relying on the PyNN neuronal simulator
abstraction layer.
"""

__author__ = "Sebastian Krach, Georg Hinkel"

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
        try:
            return type(self.__population.celltype).__name__
        except AttributeError:
            return "PopulationAssembly"

    @property
    def parameters(self):
        """
        Gets the parameters of a the population as dict
        """
        try:
            celltype = self.__population.celltype.parameter_space
            return {a: celltype[a].base_value for a in celltype.keys()}
        except AttributeError:
            return {}

    @property
    def gids(self):
        """
        Gets the global unique identifiers for this population
        """
        return [long(i) for i in self.__population.all()]

    @property
    def indices(self):
        """
        Gets the population info indices
        """
        return [i.parent.id_to_index(i) for i in self.__population.all()]
