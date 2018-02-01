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
This module contains an abstract implementation of the IBrainDevice interface and provides
default implementations.
"""
__author__ = "Sebastian Krach"

from .__DeviceGroup import DeviceGroup
from collections import OrderedDict
from hbp_nrp_cle.tf_framework import resolve_brain_variable
from hbp_nrp_cle.brainsim.BrainInterface import IBrainDevice


# PyLint does not recognize the following class as abstract and therefore complains about missing
# implementations
# pylint: disable=abstract-method

class AbstractBrainDevice(IBrainDevice):
    """
    This class provides basic, shared functionality which are common for all brain device
    implementations.
    """

    default_parameters = {
        # Subclasses should override the field and set their parameters accordingly

        # The parameters which are specified here can be reset by specifying
        #   the respective parameters on construction.
    }

    fixed_parameters = {
        # Subclasses should override the field and set their parameters accordingly

        # The parameters which are specified here are not allowed to be reset by specifying
        #   the parameters on construction.
    }

    def __init__(self, **params):
        super(AbstractBrainDevice, self).__init__()

        self._parameters = self.__class__.fixed_parameters.copy()
        self._parameters.update(self.get_parameter_defaults())
        self.verify_parameters(params)
        self._update_parameters(params)

    # pylint: disable=unused-argument
    @classmethod
    def create_new_device(cls, population, **params):
        """
        Returns a new instance of the concrete implementation of the brain device.

        :param params: additional parameters which are passed to the device constructor
        :param population: The population for which the device should be created
        :return: a new instance of the concrete device
        """
        return cls(**params)

    @classmethod
    def create_new_device_group(cls, populations, params):
        """
        Returns a new device group instance for the concrete implementation of this brain device.

        :param params: additional parameters which are passed to the constructor of the nested
            devices. For each parameter either the value can be supplied, or a list of values,
            one for each nested device.
        :param populations: The populations for which the device should be created
        :return: a new device group containing the created nested devices
        """
        return DeviceGroup.create_new_device_group(populations, cls, params)

    def reset(self, transfer_function_manager):
        """
        Resets the device

        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
        """
        return self

    def get_parameters(self, *params):
        """
        This method allows to retrieve a dictionary containing device configuration parameters.
        It is possible to retrieve only a subset of the parameters and also to conduct a suitable
        parameter remapping using the subset parameter.

        The device specific parameters are build from the fixed parameters, the parameters which
        were specified on creation of the device or the default value if no the parameter was not
        specified.

        Given that the respective parameters are contained in the device configuration dictionary

        would return a dictionary as depicted below:

        .. code-block:: python

            get_parameters("param1", "param2")
            result = {"param1": value_of_param1, "param2": value_of_param2}

        Parameter remapping can be achieved by specifying a subset dictionary or by specifying
        single remappings as tuple. Both would return a dictionary as depicted below (the key
        "param1" is remapped to "p1", while "param2" is not changed):

        .. code-block:: python

            get_parameters({"p1": "param1", "param2": "param2"})
            get_parameter(("p1", "param1"), "param2")
            result = {"p1": value_of_param1, "param2": value_of_param2}

        :param params: keys of the parameters which should be returned. Each parameter can be
            provided as tuple (new_key, key) or multiple remappings can be provided as dictionary
            {new_key: key}.
        :return: a dictionary containing the desired configuration parameters
        """
        if not params:
            result = dict(self._parameters)
        else:
            result = OrderedDict()
            for p in params:
                if isinstance(p, dict):
                    for (k, v) in p.items():
                        result[k] = self._parameters[v]
                if isinstance(p, tuple):
                    (k, v) = p
                    result[k] = self._parameters[v]
                else:
                    result[p] = self._parameters[p]
        for k in result:
            result[k] = resolve_brain_variable(result[k])
        return result

    @classmethod
    def get_parameter_defaults(cls):
        """
        The method returns a copy of the default parameter dictionary ("default_parameters"). It
        can be overridden in subclasses if a more flexible approach is necessary.

        :return: A dictionary containing device parameter defaults
        """
        return cls.default_parameters.copy()

    @classmethod
    def verify_parameters(cls, params):
        """
        Checks whether the provided parameters are suitable for the device that is constructed

        :param params: the dictionary of parameters
        """
        defaults = cls.get_parameter_defaults()
        for param in params:
            if not param in defaults:
                raise AttributeError('The current device (%s) does not support the setting "%s"'
                                     % (cls, param))

    def _update_parameters(self, params):
        """
        This method updates the device parameter dictionary with the provided parameter
        dictionary. The dictionary has to be validated before as this method assumes it to be
        correct.

        Overriding subclasses can provide additional configuration parameter adaption as this
        method is called before the brain simulator devices are constructed.

        :param params: The validated parameter dictionary
        """
        self._parameters.update(params)
