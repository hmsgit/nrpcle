"""
This module contains a base class how PyNN populations are handled
"""

from pyNN.common import BasePopulation, Assembly

__author__ = "Georg Hinkel"


def is_population(population):
    """
    Determines whether the given object is a population

    :param population: The object that may be a population
    """
    return isinstance(population, BasePopulation) or isinstance(population, Assembly)


def create_view(population, sl):
    """
    Creates a view of the given population

    :param population: The base population
    :param sl: The slice of the population that represents the view
    """
    if isinstance(sl, int):
        return population[sl:(sl + 1)]
    else:
        return population[sl]
