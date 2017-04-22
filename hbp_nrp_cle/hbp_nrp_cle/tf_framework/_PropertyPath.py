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
This module contains a selector class selecting property paths
"""

__author__ = 'GeorgHinkel'

from copy import deepcopy


class PropertyPath(object):
    """
    Represents the path to a specified sub object
    """

    # pylint: disable=R0201,R0921,R0924
    def __getattr__(self, item):
        """
        Gets the attribute with the specified name

        :param item: The attribute name
        """
        return AttributePathSegment(item, self)

    def __getitem__(self, item):
        """
        Gets the item at the specified index

        :param item: The index
        """
        return IndexPathSegment(item, self)

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return "(root)"

    def select(self, root):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        """
        return root

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, deepcopy(v, memo))
        return result


class AttributePathSegment(PropertyPath):
    """
    Represents an attribute segment in a property path
    """

    # pylint: disable=R0201,R0921, R0924
    def __init__(self, name, parent):
        """
        Creates a new path segment with the given attribute name based on the given parent path

        :param name: The attribute name
        :param parent: The parent path
        """
        assert isinstance(parent, PropertyPath)
        self.__name = name
        self.__parent = parent

    @property
    def name(self):
        """
        Gets the attribute name

        :return: The attribute name
        """
        return self.__name

    @name.setter
    def name(self, value):
        """
        Sets the attribute name

        :param value: The new attribute name
        """
        self.__name = value

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return repr(self.__parent) + "." + self.name

    def select(self, root):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        """
        parent = self.__parent.select(root)
        if parent is None:
            return None
        return getattr(parent, self.name)


class IndexPathSegment(PropertyPath):
    """
    Represents an index path segment
    """

    # pylint: disable=R0201,R0921, R0924
    def __init__(self, index, parent):
        """
        Creates a new index path segment for the given index and parent path

        :param index: The index to be used to get to the child element
        :param parent: The parent path
        """
        assert isinstance(parent, PropertyPath)
        if isinstance(index, int):
            self.__index = slice(index, index + 1)
        else:
            self.__index = index
        self.__parent = parent

    @property
    def index(self):
        """
        Gets the index that should be used for this element
        """
        return self.__index

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return self.__parent.__repr__() + "[" + self.__index.__repr__() + "]"

    def select(self, root):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        """
        parent = self.__parent.select(root)
        if parent is None:
            return None
        return parent[self.__index]

    @property
    def parent(self):
        """
        Gets the parent node for this element
        """
        return self.__parent
