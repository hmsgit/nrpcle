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
'''
NengoControlAdapter.py
moduleauthor: krach@fzi.de
'''

from hbp_nrp_cle.brainsim import IBrainControlAdapter
from hbp_nrp_cle.brainsim.nengo.NengoInfo import is_population, NengoPopulationInfo
from hbp_nrp_cle.brainsim.nengo import NengoBrainLoader
from hbp_nrp_cle.cle.CLEInterface import BrainRuntimeException

import nengo

import logging
from os import path

logger = logging.getLogger(__name__)

__author__ = 'Sebastian Krach'


class NengoControlAdapter(IBrainControlAdapter):
    """
    Represents a controller object for the neuronal simulator
    """

    def __init__(self, nengo_simulation_state):
        """
        Initializes the Nengo control adapter

        :param sim: The simulator module
        """
        self.__is_initialized = False

        self._nengo_simulation_state = nengo_simulation_state
        self._params = {'dt': 0.001}

    def load_populations(self, **populations):
        """
        Load (or reload) populations into the brain

        :param populations: A dictionary indexed by population names and
          containing neuron indices. Neuron indices can be defined by a single integer,
          list of integers or python slices. Python slices can be replaced by a
          dictionary containing the 'from', 'to' and 'step' values.
        """
        if populations:
            raise NotImplementedError(
                "The Nengo Brain Adapter currently does not support selecting populations"
                " based on BIBI file attributes. You can provide named populations by"
                " having the brain module declare these as attribute.")

    def load_brain(self, brain_file, **populations):
        """
        Loads the neuronal network contained in the given file

        :param brain_file: The path to the neuronal network file
        """
        extension = path.splitext(brain_file)[1]

        if extension == ".py":
            self._nengo_simulation_state.load_brain(brain_file)
            import hbp_nrp_cle.tf_framework.config as tf_config
            tf_config.brain_populations = {}
            NengoBrainLoader.setup_access_to_population(
                tf_config.brain_root, *self.get_populations())
        else:
            msg = "Neuronal network format {0} not supported".format(extension)
            raise Exception(msg)

    def initialize(self, **sim_params):
        """
        Initializes the neuronal simulator. All additional keyword parameters are passed to
        the constructor of the simulator.

        :return: True if the simulator is initialized, otherwise False
        """
        if not self.__is_initialized:
            self._params.update(sim_params)
            self._nengo_simulation_state.initialize(lambda brain_root: nengo.Simulator(
                brain_root, **self._params))

            self.__is_initialized = True
            logger.info("neuronal simulator initialized")
        else:
            logger.warn(
                "trying to initialize an already initialized controller")
        return self.__is_initialized

    @staticmethod
    def _is_population(candidate):
        """
        Determines whether the candidate is a population

        :param candidate: The candidate
        """
        return is_population(candidate)

    @staticmethod
    def _create_population_info(population, name):
        """
        Creates a population info object for the given population

        :param population: The population
        :param name: The name of the population
        """
        # Currently the Nengo Brain Adapter implementation does not provide population specific
        # information. This might need to be enhanced once visualization requirements are clear.
        parameters = {}
        return NengoPopulationInfo(population, name, parameters)

    def __find_all_populations(self, candidate, member_name, populations):
        """
        Finds all populations under the given object and adds them to the list of populations

        :param candidate: The object tree
        :param member_name: The base member name
        :param populations: The list of populations
        """
        if self._is_population(candidate):
            populations.append(
                self._create_population_info(candidate, member_name))
        elif isinstance(candidate, list):
            for index, cand in enumerate(candidate):
                self.__find_all_populations(
                    cand, '{name}[{i}]'.format(name=member_name, i=index), populations)

    def get_populations(self):
        """
        Gets an information about the populations currently available

        :return: A list of population infos
        """
        import hbp_nrp_cle.tf_framework.config as config
        populations = []
        for member in dir(config.brain_root):
            candidate = getattr(config.brain_root, member)
            self.__find_all_populations(candidate, member, populations)
        return populations

    @property
    def is_initialized(self):  # pragma: no cover
        """
        Gets a value indicating whether initialize has been called
        """
        return self.__is_initialized

    def is_alive(self):  # -> bool:  # pragma: no cover
        """
        Gets a status whether the neuronal simulator is still alive

        :return: True if the simulator is alive, otherwise False
        """
        return not self._nengo_simulation_state.simulator.closed

    def run_step(self, dt):  # -> None:
        """
        Runs the neuronal simulator for the given amount of simulated time

        :param dt: the simulated time in milliseconds
        """

        try:
            _sim = self._nengo_simulation_state.simulator

            for _ in range(int(dt / (_sim.dt * 1000))):
                _sim.step()
        except Exception as e:
            raise BrainRuntimeException(str(e))

    def shutdown(self):  # pragma: no cover
        """
        Shuts down the neuronal simulator
        """
        self.__is_initialized = False
        logger.info("neuronal simulator ended")

    def reset(self):  # pragma: no cover
        """
        Resets the neuronal simulator
        """
        self._nengo_simulation_state.reset_simulator()
        logger.info("neuronal simulator reset")

    # pylint: disable=no-self-use
    def get_Timeout(self):  # pragma: no cover
        """
        returns The maximum amount of time (in seconds) to wait for the end of this step
        """
        return 5
