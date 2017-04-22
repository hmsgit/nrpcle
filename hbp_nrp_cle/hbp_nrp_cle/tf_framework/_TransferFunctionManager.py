# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
This module contains the implementation of a transfer functions manager
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter
from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter
from ._TransferFunctionInterface import ITransferFunctionManager
from ._PropertyPath import PropertyPath
from . import config
import itertools
import logging
import time

logger = logging.getLogger(__name__)


class TransferFunctionManager(ITransferFunctionManager):
    """
    Represents a transfer functions node
    """

    def __init__(self):  # -> None:
        """
        Creates a new transfer functions node
        """

        self.__n2r = []
        self.__r2n = []
        self.__robotAdapter = None
        self.__nestAdapter = None
        self.__initialized = False
        self.__global_data = {}

    @property
    def n2r(self):  # -> list:
        """
        Gets a list of transfer functions from the neuronal simulator to the world simulation
        """
        return self.__n2r

    @property
    def r2n(self):  # -> list:
        """
        Gets a list of transfer functions from the world simulator to the neuronal simulation
        """
        return self.__r2n

    @property
    def global_data(self):
        """
        Gets the global variable storage dictionary for the transfer functions

        :return: the dictionary mapping variable name to value
        """
        return self.__global_data

    @staticmethod
    def _run_tf(tf, t):  # -> None:
        """
        Runs the transfer functions for the given point in simulation time

        :param tf: the transfer function
        :param t: The simulation time
        """

        start = time.time()
        tf.run(t)
        tf.elapsed_time += time.time() - start

    def run_neuron_to_robot(self, t):  # -> None:
        """
        Runs the transfer functions from the neuronal simulator towards the robot

        :param t: The simulation time
        """
        for _n2r in self.__n2r:
            TransferFunctionManager._run_tf(_n2r, t)

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation

        :param t:  The simulation time
        """
        for _r2n in self.__r2n:
            TransferFunctionManager._run_tf(_r2n, t)

    @staticmethod
    def __select_neurons(neurons):
        """
        Selects the neurons represented by the given property path

        :param neurons:
        """
        if isinstance(neurons, PropertyPath):
            return neurons.select(config.brain_root)
        if isinstance(neurons, list):
            for i in range(0, len(neurons)):
                neurons[i] = TransferFunctionManager.__select_neurons(neurons[i])
        return neurons

    @property
    def robot_adapter(self):  # -> IRobotCommunicationAdapter:
        """
        Gets or sets the adapter to the world simulation
        """
        return self.__robotAdapter

    @robot_adapter.setter
    def robot_adapter(self, robot_adapter):  # -> None:
        """
        Sets the robot adapter

        :param robot_adapter: The new robot adapter
        """
        if self.__initialized:
            raise Exception("Cannot exchange robot adapter after node has been initialized!")
        else:
            if not isinstance(robot_adapter, IRobotCommunicationAdapter):
                raise Exception("The given object is not a valid robot adapter!")
            self.__robotAdapter = robot_adapter

    @property
    def initialized(self):
        """
        Determines whether the current tf manager has already been initialized

        :return: True, if the TF manager has been initialized, otherwise False
        """
        return self.__initialized

    @property
    def brain_adapter(self):  # -> IBrainCommunicationAdapter:
        """
        Gets or sets the adapter to the brain simulation
        """
        return self.__nestAdapter

    @brain_adapter.setter
    def brain_adapter(self, nest_adapter):  # -> None:
        """
        Sets the nest adapter

        :param nest_adapter: The new brain simulation adapter
        """
        if self.__initialized:
            raise Exception("Cannot exchange brainsim adapter after node has been initialized!")
        else:
            if not isinstance(nest_adapter, IBrainCommunicationAdapter):
                raise Exception("The given object is not a valid brain communication adapter")
            self.__nestAdapter = nest_adapter

    def transfer_functions(self):
        """
        Gets a list of transfer functions managed by this instance

        :return: A list of transfer functions
        """
        return self.__n2r + self.__r2n

    def initialize_tf(self, tf):
        """
        Initializes the given transfer function

        This method is used if a transfer function is replaced after initialization of the tf
        manager

        :param name: The transfer function
        """
        logger.info("Initialize transfer function " + repr(tf))
        tf.check_params()
        tf.elapsed_time = 0.0

        if hasattr(tf, 'topic') and tf.topic is not None:
            saved = tf.topic
            tf.topic = self.__robotAdapter.register_publish_topic(saved)
            tf.topic.spec = saved

        for i in range(1, len(tf.params)):
            param = tf.params[i]
            tf.params[i] = tf.params[i].create_adapter(self)
            tf.params[i].spec = param
            tf.__dict__[param.name] = tf.params[i]

        tf.initialize(self, True, True)

    def initialize(self, name):
        """
        Initializes the transfer Function node with the given name

        :param name: The name for this transfer function node
        """
        if self.__initialized:
            return

        logger.info("Initialize transfer functions node " + name)

        if not isinstance(self.__nestAdapter, IBrainCommunicationAdapter):
            raise Exception("The brain adapter is configured incorrectly")
        if not isinstance(self.__robotAdapter, IRobotCommunicationAdapter):
            raise Exception("The robot adapter is configured incorrectly")

        # Wire transfer functions
        for tf in itertools.chain(self.__r2n, self.__n2r):
            self.initialize_tf(tf)

        # Initialize dependencies
        self.__nestAdapter.initialize()
        self.__robotAdapter.initialize(name)
        self.__initialized = True

    def __reset_tf(self, tf):
        """
        Resets the given transfer function
        """
        tf.elapsed_time = 0.0
        tf.check_params()
        if hasattr(tf, "topic") and tf.topic is not None:
            tf.topic = tf.topic.reset(self)
        for i in range(1, len(tf.params)):
            param = tf.params[i]
            reset_value = param.reset(self)
            if param is not reset_value:
                tf.params[i] = reset_value
                for k in tf.__dict__:
                    if tf.__dict__[k] is param:
                        tf.__dict__[k] = reset_value

    def reset(self):  # -> None:
        """
        Resets the transfer functions
        """
        logger.info("Resetting transfer functions")

        # Wire transfer functions from neuronal simulation to world simulation
        for tf in itertools.chain(self.__r2n, self.__n2r):
            self.__reset_tf(tf)

    def shutdown(self):
        """
        Shuts down the Transfer Function manager
        """
        del self.__n2r[:]
        del self.__r2n[:]
        del self.__global_data[:]
        self.__initialized = False

    def hard_reset_brain_devices(self):
        """
        Performs a hard reset for the devices that connect with the neuronal simulation
        """
        if not self.initialized:
            return

        self.brain_adapter.shutdown()
        self.brain_adapter.initialize()

        for tf in itertools.chain(self.__r2n, self.__n2r):
            for i in range(1, len(tf.params)):
                spec = tf.params[i].spec
                if spec.is_brain_connection:
                    tf.params[i] = spec.create_adapter(self)
                    tf.params[i].spec = spec
                    tf.__dict__[spec.name] = tf.params[i]
            tf.initialize(self, True, False)

    def hard_reset_robot_devices(self):
        """
        Performs a hard reset for the devices that connect with the simulated robot
        """
        if not self.initialized:
            return

        self.robot_adapter.shutdown()
        self.robot_adapter.initialize()

        for tf in itertools.chain(self.__r2n, self.__n2r):
            for i in range(1, len(tf.params)):
                spec = tf.params[i].spec
                if spec.is_robot_connection:
                    tf.params[i] = spec.create_adapter(self)
                    tf.params[i].spec = spec
                    tf.__dict__[spec.name] = tf.params[i]
            tf.initialize(self, False, True)
