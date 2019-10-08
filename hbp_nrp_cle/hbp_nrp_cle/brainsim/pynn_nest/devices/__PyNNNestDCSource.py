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
This module contains the nest implementation of the DC source for nest
"""
from hbp_nrp_cle.brainsim.pynn.devices import PyNNDCSource
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice, \
    create_transformation, PyNNNestDeviceGroup
import pyNN.nest as nestsim

__author__ = 'Georg Hinkel, DimitriProbst, Sebastian Krach'


class PyNNNestDCSource(PyNNNestDevice, PyNNDCSource):
    """
    Represents a direct current generator
    """

    transformations = {
        "amplitude": create_transformation("amplitude", 1000.0)
    }

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self._parameters["amplitude"]

    def sim(self):
        """
        Gets the simulator module to use
        """
        return nestsim

    # Pylint does not really recognize property overrides
    # pylint: disable=arguments-differ
    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        if self.amplitude != value:
            self._parameters["amplitude"] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {"amplitude": 1000.0 * value})

    # pylint: disable=unused-argument
    @staticmethod
    def _should_use_integrated_dc_gen(population, params):
        """
        Predicate used for deciding whether an integrated NEST generator should be used

        :param population: The population for which the device should be created
        :param params: additional parameters which are passed to the device constructor
        :return: True if an optimized generator should be used, False otherwise
        """

        # TODO: the call below to nest.GetStatus blocks execution in the MPI mode. A temporary
        # solution has been to disable the possibility of using IntegratedNestDCCurrentGenerator.
        # A more permanent solution would involve to figure out how to evaluate the condition in the
        # return statement without making the nest call. Since the cases in which this optimized
        # generator can be used are low and involves setting a device parameter, "parrot", which
        # to my knowledge is not documented, for now just the commented code is left as reference.
        # Another solution to clean up is to remove this function and the
        # IntegratedNestDCCurrentGenerator class, since it's not used.

        # nest_status = nest.GetStatus([population.all_cells[0]])
        # return 'I_e' in nest_status[0].keys() and \
        #        not params.get("parrot", False)

        return False

    @classmethod
    def create_new_device(cls, population, **params):
        """
        Returns a new instance of the concrete implementation of the brain device

        :param params: additional parameters which are passed to the device constructor
        :param population: The population for which the device should be created
        :return: a new instance of the concrete device
        """
        # NOTE: class disabled as currently not reachable. See TODO comment above
        # if cls._should_use_integrated_dc_gen(population, params):
        #     new_device = IntegratedNestDCCurrentGenerator(**params)
        # else:
        new_device = PyNNNestDCSource(**params)

        return new_device

    @classmethod
    def create_new_device_group(cls, populations, params):

        # the populations list is homogeneous,
        # so just use the first one to decide the type
        # NOTE: class disabled as currently not reachable. See TODO comment above
        # if cls._should_use_integrated_dc_gen(populations[0], params):
        #     generator_type = IntegratedNestDCCurrentGenerator
        # else:
        generator_type = cls

        return PyNNNestDeviceGroup.create_new_device_group(populations, generator_type, params)

    @classmethod
    def get_parameter_defaults(cls):
        """
        The method returns a copy of the default parameter dictionary ("default_parameters"). It
        can be overridden in subclasses if a more flexible approach is necessary.

        :return: A dictionary containing device parameter defaults
        """
        defaults = cls.default_parameters.copy()
        defaults["parrot"] = False
        return defaults

# NOTE: class disabled as currently not reachable. See TODO comment above
# class IntegratedNestDCCurrentGenerator(PyNNNestDevice, PyNNDCSource):
#     """
#     This class implements a current generator by reusing the I_e parameter in certain nest models
#     """
#
#     transformations = {
#         "amplitude": create_transformation("I_e", 1000.0)
#     }
#
#     @property
#     def amplitude(self):
#         """
#         Returns the amplitude of the current
#         """
#         return self._parameters["amplitude"]
#
#     def sim(self):
#         """
#         Gets the simulator module to use
#         """
#         return nestsim
#
#     # Pylint does not really recognize property overrides
#     # pylint: disable=arguments-differ
#     @amplitude.setter
#     def amplitude(self, value):
#         """
#         Sets the amplitude of the current
#
#         :param value: float
#         """
#         if self.amplitude != value:
#             self._parameters["amplitude"] = value
#             self.SetStatus(list(self._generator.all_cells), {"I_e": 1000.0 * value})
#
#     def create_device(self):
#         """
#         This method usually creates a new device, but in this case, we do not need one
#         """
#         pass
#
#     def connect(self, neurons):
#         """
#         Connects the device to the given neuron population
#         """
#         self._generator = neurons
#
#     @property
#     def device_id(self):
#         """
#         Gets the device id this device is connected to
#         """
#         return self._generator.all_cells[0]
