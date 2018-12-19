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
NengoSimulationState.py
moduleauthor: krach@fzi.de
'''

from hbp_nrp_cle.brainsim.common import InternalBrainException
from hbp_nrp_cle.brainsim.nengo import NengoBrainLoader

import nengo
import logging

logger = logging.getLogger(__name__)

__author__ = 'Sebastian Krach'


class NengoSimulationState(object):
    """
    Holds information about the Nengo simulation state
    """

    def __init__(self):
        """
        Init
        """
        self._simulator = None
        self._simulator_factory = None
        self._root_network = None

    def initialize(self, simulator_factory):
        """
        Initialize
        """
        self._simulator_factory = simulator_factory

    def load_brain(self, network_file):
        """
        Loads the brain model specified in the given Python script

        :param network_file: The Python file containing the network
        """
        import hbp_nrp_cle.tf_framework.config as tf_config
        tf_config.brain_root = NengoBrainLoader.load_py_network(network_file)

        self._root_network = nengo.Network()

        with self._root_network:
            nengo.Network.add(tf_config.brain_root.circuit)

        logger.info("Saving brain source")

        with open(network_file) as source:
            tf_config.brain_source = source.read()

        logger.info("Resetting Nengo simulator")
        self._simulator = None

    @property
    def initialized(self):
        """
        Returns whether the simulation state is initialized.
        :return: True, iff a simulation factory has been set.
        """
        return self._simulator_factory is not None

    @property
    def simulator(self):
        """
        Returns the Nengo simulator instance. The simulator is initialized on first usage.
        In case the brain model has changed a new simulator is instantiated.
        :return:
        """
        if not self._simulator:
            logger.info("Initializing new Nengo simulator instance")
            self._simulator = self._simulator_factory(self.brain_root)
        return self._simulator

    @property
    def simulation_data(self):
        """
        Returns the simulation data
        """
        return self._simulator.data

    @property
    def brain_root(self):
        """
        Returns the brain root
        """
        return self._root_network

    def __enter__(self):
        self._root_network.__enter__()
        return self

    def __exit__(self, dummy_exc_type, dummy_exc_value, dummy_tb):
        self._root_network.__exit__(dummy_exc_type, dummy_exc_value, dummy_tb)

    def delete_from_brain(self, device):
        """
        Remove the brain device from the underlying root network structure.
        """
        network = self._root_network
        for cls in type(device).__mro__:
            if cls in network.objects:
                logger.info("Removing device %r of type %r from brain root network", device, cls)
                network.objects[cls].remove(device)
                break
        else:
            raise InternalBrainException("Removing objects of type {:!r} is not supported by the"
                                         " employed Nengo version".format(type(device).__name__))

        # Reset the current simulator instance as network model changed.
        self._simulator = None

    def reset_simulator(self):
        """
        Resets the Nengo simulator
        """
        self._simulator = None
