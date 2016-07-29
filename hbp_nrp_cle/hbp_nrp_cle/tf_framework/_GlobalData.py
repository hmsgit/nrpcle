"""
This module contains the mapping of transfer function local or global variables to input parameters
"""
__author__ = 'Sebastian Krach'

from ._MappingSpecification import ParameterMappingSpecification

import abc
import logging
import pyretina

logger = logging.getLogger(__name__)

GLOBAL = "global_scope"
TRANSFER_FUNCTION_LOCAL = "transfer_function_scope"


class PyRetinaWrapper(object):
    """
    Class to wrap a retina object
    """

    def __init__(self, retina):
        """
        Wraps a retina object

        :param retina: a retina object
        """
        self.__retina = retina

    # pylint: disable=unused-argument
    def reset(self, transfer_function_manager):
        """
        Reset a retina
        """
        return self

    def update(self, image):
        """
        Update a retina with respect to an image

        :param image: a cv2 image
        """
        return self.__retina.update(image)

    def getValue(self, x, y, layer_name):
        """
        Read activation of a retina neuron

        :param x: the x coordinate of the retina neuron
        :param y: the y coordinate of the retina neuron
        :param layer_name: the name of the layer of the retina neuron
        """
        return self.__retina.getValue(x, y, layer_name)


class MapVariable(ParameterMappingSpecification):
    """
    Class to map transfer function local or global variables to transfer function parameters
    """

    def __init__(self, parameter_name, global_key=None, initial_value=None,
                 scope=TRANSFER_FUNCTION_LOCAL):  # -> None:
        """
        Maps a parameter to a variable in the specified scope (per-default: the transfer function)
        and if the variable does not yet exist initializes it with the provided value.

        :param parameter_name: the name of the parameter
        :param global_key: (Optional) the name of the variable in the global scope, if not specified
        it is assumed to be equal to the parameter name
        :param initial_value: (Optional) the value for the parameter
        :param scope: (Optional) the scope of validity for the variable
        """
        super(MapVariable, self).__init__(parameter_name)
        if global_key:
            self.__global_key = global_key
        else:
            self.__global_key = parameter_name
        self.__initial_value = initial_value
        self.__scope = scope

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        if self.__scope == GLOBAL:
            return GlobalDataReference(self.name, self.__global_key,
                                       self.__initial_value, transfer_function_manager.global_data)
        elif self.__scope == TRANSFER_FUNCTION_LOCAL:
            return LocalDataReference(self.name, self.__initial_value)
        else:
            raise AttributeError("The specified parameter scope is not valid.")


class MapRetina(ParameterMappingSpecification):
    """
    Class to map transfer function local or global retina config file
    to transfer function parameters
    """

    def __init__(self, parameter_name, config=None):
        """
        Maps a parameter to a retina in the transfer function scope
        and if the variable does not yet exist initializes it with the provided config file.

        :param parameter_name: the name of the parameter
        :param config: the selected config file
        """
        self.retina_config_file = config
        super(MapRetina, self).__init__(parameter_name)

    # pylint: disable=unused-argument
    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        retina = pyretina.Retina(self.retina_config_file)
        return PyRetinaWrapper(retina)


class DataReference(object):
    """
    Abstract super class for transfer function variables. Must be implemented by subclasses which
    support storing and providing of parameters to transfer functions.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name, initial_value):
        self.__name = name
        self.__initial_value = initial_value

    @property
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__name

    @property
    def value(self):
        """
        Gets the value associated with this parameter. Has to be provided by the implementing
        subclass.

        :return: the parameter value
        """
        return self._value_getter()

    @value.setter
    def value(self, _val):
        """
        Sets the value associated with this parameter. Has to be provided by the implementing
        subclass.

        :param _val: The new value
        """
        self._value_setter(_val)

    @abc.abstractmethod
    def _value_getter(self):
        """
        Gets the value associated with this parameter. Abstract method to force subclasses to
        provide functionality for value property.

        :return: The value.
        """
        raise Exception("Must be overwritten by instantiating subclass.")

    @abc.abstractmethod
    def _value_setter(self, _val):
        """
        Sets the value associated with this parameter. Abstract method to force subclasses to
        provide functionality for value property.

        :param _val: The new value
        """
        raise Exception("Must be overwritten by instantiating subclass.")

    # pylint: disable=unused-argument
    def reset(self, transfer_function_manager):
        """
        Resets the GlobalDataReference instance to its initial value.

        :param transfer_function_manager: The transfer function manager the tf is contained in
        :return: The reset instance
        """
        self.value = self.__initial_value
        return self


class LocalDataReference(DataReference):
    """
    This class acts as concrete mapping between an input parameter to a transfer function and the
    parameter value stored locally for one transfer function.
    """

    def __init__(self, name, initial_value):
        super(LocalDataReference, self).__init__(name, initial_value)
        self.__value = initial_value

    def _value_getter(self):
        """
        Gets the value associated with this parameter
        """
        return self.__value

    def _value_setter(self, _val):
        """
        Sets the value associated with this parameter

        :param _val: the new value
        """
        self.__value = _val


class GlobalDataReference(DataReference):
    """
    This class acts as concrete mapping between an input parameter to a transfer function and the
    dictionary storing the actual value.
    """

    def __init__(self, name, global_key, initial_value, data_dictionary):
        """
        Creates a new parameter mapping.

        :param name: the name of the parameter
        :param data_dictionary: the dictionary where to look up the value and store it to
        """

        super(GlobalDataReference, self).__init__(name, initial_value)
        self.__global_key = global_key
        self.__data_dict = data_dictionary

        if self.__global_key not in self.__data_dict:
            self.reset(None)

    def _value_getter(self):
        """
        Gets the value associated with this parameter
        """
        return self.__data_dict[self.__global_key]

    def _value_setter(self, _val):
        """
        Sets the value associated with this parameter

        :param _val: the new value
        """
        self.__data_dict[self.__global_key] = _val
