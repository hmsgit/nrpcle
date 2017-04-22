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
This module contains the supported neuron selectors
"""

__author__ = 'GeorgHinkel'


class NeuronSelector(object):
    """
    The base class for neuron selectors
    """

    def __init__(self):
        """
        Creates a new neuron selector
        """
        pass

    def select(self, root):
        """
        Selects the neurons based on the given brain root
        :param root: The brain root element
        :return: A list of neurons based on the mapper
        """
        raise NotImplementedError("This method must be overridden")


class MapNeuronSelector(NeuronSelector):
    """
    The mapping operator to map a
    """

    def __init__(self, neuron_range, mapping):
        """
        Creates a new neuron mapping
        :param neuron_range: The range of neurons
        :param mapping: The mapping function
        """
        super(MapNeuronSelector, self).__init__()
        if isinstance(neuron_range, int):
            neuron_range = range(0, neuron_range)
        self.__neuron_range = neuron_range
        self.__mapping = mapping

    @property
    def neuron_range(self):
        """
        The iterator on which the neurons are created
        """
        return self.__neuron_range

    @property
    def mapping(self):
        """
        The mapping function for the device groups
        """
        return self.__mapping

    def __repr__(self):
        """
        Gets a string representation of the neuron selector
        """
        return "mapping " + repr(self.mapping)

    def select(self, root):
        """
        Selects the neurons based on the given brain root
        :param root: The brain root element
        :return: A list of neurons based on the mapper
        """
        result = []
        for item in self.neuron_range:
            neurons = self.mapping(item).select(root)
            if isinstance(neurons, list):
                result = result + neurons
            else:
                result.append(neurons)
        return result


class ChainNeuronSelector(NeuronSelector):
    """
    Represents a chain of neuron selectors
    """

    def __init__(self, selectors):
        """
        Creates a new chain selector
        :param selectors: A list of selectors
        """
        super(ChainNeuronSelector, self).__init__()
        self.__selectors = selectors

    @property
    def selectors(self):
        """
        The list of selectors
        """
        return self.__selectors

    def __repr__(self):
        """
        Gets a string representation of the neuron selector
        """
        return '[%s]' % (','.join(repr(s) for s in self.selectors),)

    def select(self, root):
        """
        Selects the neurons based on the given brain root
        :param root: The brain root element
        """
        result = []
        for selector in self.selectors:
            selected = selector.select(root)
            if isinstance(selected, list):
                result = result + selected
            else:
                result.append(selected)
        return result
