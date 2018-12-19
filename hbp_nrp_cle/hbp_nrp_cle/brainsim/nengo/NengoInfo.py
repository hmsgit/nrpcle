# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains a base class how Nengo populations are handled
"""

from nengo import Ensemble
from hbp_nrp_cle.brainsim.BrainInterface import PopulationInfo


__author__ = "Sebastian Krach"


def is_population(population):  # pragma: no cover
    """
    Determines whether the given object is a population

    :param population: The object that may be a population
    """
    return isinstance(population, Ensemble)


def create_view(population, sl):  # pragma: no cover
    """
    Creates a view of the given population

    :param population: The base population
    :param sl: The slice of the population that represents the view
    """
    if isinstance(sl, int):
        return population[sl:(sl + 1)]
    else:
        return population[sl]


class NengoPopulationInfo(PopulationInfo):  # pragma: no cover
    """
    The Nengo implementation of populations
    """

    def __init__(self, population, name, parameters):
        """
        Creates a new Nengo population based on the given population and the given name

        :param population: The underlying Nengo ensemble
        :param name: The name for the population
        :param parameters: The population parameters
        """
        self.__population = population
        self.__name = name
        self.__parameters = parameters
        self.__gids = range(len(population.neurons))
        self.__indices = self.__gids

    @property
    def name(self):
        """
        Gets the population name
        """
        return self.__name

    @property
    def population(self):
        """
        Gets the population itself
        """
        return self.__population

    @property
    def celltype(self):
        """
        Gets the celltype of the population
        """
        try:
            return type(self.__population.neuron_type).__name__
        except AttributeError:
            return "PopulationAssembly"

    @property
    def parameters(self):
        """
        Gets the parameters of a the population as dict
        """
        return self.__parameters

    @property
    def gids(self):
        """
        Dummy implementation as Nengo does use GIDS
        :return: Array of consecutive ids with a size equal to the number of neurons,
        starting with id 0.
        """
        return self.__gids

    @property
    def indices(self):
        """
        Dummy implementation as Nengo does use Indices
        :return: Array of consecutive ids with a size equal to the number of neurons,
        starting with id 0.
        """
        return self.__indices
