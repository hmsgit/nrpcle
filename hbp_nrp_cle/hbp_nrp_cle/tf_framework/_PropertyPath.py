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
import inspect


class PropertyPath(object):
    """
    Represents the path to a specified sub object
    """

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

    def __len__(self):
        """
        Gets the length of the current path
        """
        return CallSegment(AttributePathSegment('__len__', self))

    @staticmethod
    def _make_path(value):
        """
        Converts the
        :param value:
        :return:
        """
        if isinstance(value, PropertyPath):
            return value
        else:
            return ConstantSegment(value)

    def __add__(self, other):
        """
        Performs a sum

        :param other: Another item
        :return: The sum of both
        """
        return CallSegment(AttributePathSegment('__add__', self), PropertyPath._make_path(other))

    def __radd__(self, other):
        """
        Performs a sum

        :param other: Another item
        :return: The sum of both
        """
        return CallSegment(AttributePathSegment('__add__', PropertyPath._make_path(other)), self)

    def __mul__(self, other):
        """
        Performs a product

        :param other: Another item
        :return: The product of both
        """
        return CallSegment(AttributePathSegment('__mul__', self), PropertyPath._make_path(other))

    def __rmul__(self, other):
        """
        Performs a product

        :param other: Another item
        :return: The product of both
        """
        return CallSegment(AttributePathSegment('__mul__', PropertyPath._make_path(other)), self)

    def __and__(self, other):
        """
        Performs an and operation

        :param other: Another item
        :return: The and of both
        """
        return CallSegment(AttributePathSegment('__and__', self), PropertyPath._make_path(other))

    def __rand__(self, other):
        """
        Performs an and operation

        :param other: Another item
        :return: The and of both
        """
        return CallSegment(AttributePathSegment('__and__', PropertyPath._make_path(other)), self)

    def __or__(self, other):
        """
        Performs an and operation

        :param other: Another item
        :return: The and of both
        """
        return CallSegment(AttributePathSegment('__or__', self), PropertyPath._make_path(other))

    def __ror__(self, other):
        """
        Performs an and operation

        :param other: Another item
        :return: The and of both
        """
        return CallSegment(AttributePathSegment('__or__', PropertyPath._make_path(other)), self)

    def __xor__(self, other):
        """
        Performs an and operation

        :param other: Another item
        :return: The and of both
        """
        return CallSegment(AttributePathSegment('__xor__', self), PropertyPath._make_path(other))

    def __rxor__(self, other):
        """
        Performs an and operation

        :param other: Another item
        :return: The and of both
        """
        return CallSegment(AttributePathSegment('__xor__', PropertyPath._make_path(other)), self)

    def __div__(self, other):
        """
        Performs a division

        :param other: Another item
        :return: The division of both
        """
        return CallSegment(AttributePathSegment('__div__', self), PropertyPath._make_path(other))

    def __rdiv__(self, other):
        """
        Performs a division

        :param other: Another item
        :return: The division of both
        """
        return CallSegment(AttributePathSegment('__div__', PropertyPath._make_path(other)), self)

    def __eq__(self, other):
        """
        Checks whether the given path is equivalent to the

        :param other: Another item
        :return: The sum of both
        """
        return EqualsSegment(self, PropertyPath._make_path(other))

    def __ne__(self, other):
        """
        Checks whether the given path is equivalent to the

        :param other: Another item
        :return: The sum of both
        """
        return NotEqualsSegment(self, PropertyPath._make_path(other))

    def __ge__(self, other):
        """
        Checks that the current path item is greather than the specified one

        :param other: Another item
        :return: Whether the current item is greater or equal
        """
        return GreaterThanEqualsSegment(self, PropertyPath._make_path(other))

    def __le__(self, other):
        """
        Checks that the current path item is less than the specified one

        :param other: Another item
        :return: Whether the current item is less or equal
        """
        return LessThanEqualsSegment(self, PropertyPath._make_path(other))

    def __gt__(self, other):
        """
        Checks that the current path item is greather than the specified one

        :param other: Another item
        :return: Whether the current item is greater or equal
        """
        return GreaterThanSegment(self, PropertyPath._make_path(other))

    def __lt__(self, other):
        """
        Checks that the current path item is less than the specified one

        :param other: Another item
        :return: Whether the current item is less or equal
        """
        return LessThanSegment(self, PropertyPath._make_path(other))

    def __sub__(self, other):
        """
        Performs a subtraction

        :param other: Another item
        :return: The difference of both
        """
        return CallSegment(AttributePathSegment('__sub__', self), PropertyPath._make_path(other))

    def __rsub__(self, other):
        """
        Performs a subtraction

        :param other: Another item
        :return: The difference of both
        """
        return CallSegment(AttributePathSegment('__sub__', PropertyPath._make_path(other)), self)

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return "(root)"

    # pylint: disable=unused-argument, no-self-use
    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
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

    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
        """
        parent = self.__parent.select(root, bca)
        if parent is None:
            return None
        return getattr(parent, self.name)


class ConstantSegment(PropertyPath):
    """
    Represents a constant segment in a property path
    """

    def __init__(self, value):
        """
        Creates a new value path segment
        :param value:
        """
        self.__value = value

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return repr(self.__value)

    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
        """
        return self.__value


class CallSegment(PropertyPath):
    """
    Represents an call segment in a property path
    """

    def __init__(self, parent, *args):
        """
        Creates a new path segment with the given attribute name based on the given parent path

        :param parent: The parent path
        :param args: The list of arguments
        """
        assert isinstance(parent, PropertyPath)
        self.__parent = parent
        self.__args = args

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return repr(self.__parent) + "(" + ", ".join(map(repr, self.__args)) + ")"

    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
        """
        parent = self.__parent.select(root, bca)
        if parent is None:
            return None
        args = [arg.select(root, bca) for arg in self.__args]
        return parent(*args)


class EqualsSegment(PropertyPath):
    """
    Represents a segments that checks whether two path elements are equal

    This is necessary because many builtin classes such as int do not implement __eq__
    """
    def __init__(self, left, right):
        """
        Creates a new equals segment

        :param left: The left path
        :param right: The right path
        """
        self.__left = left
        self.__right = right

    @classmethod
    def operator_name(cls):
        """
        The operator name

        :returns: A string representation of the operator represented by the current element
        """
        return "=="

    @classmethod
    def operate(cls, left, right):
        """
        Actually perform the operator

        :param left: The left operand
        :param right: The right operand
        """
        return left == right

    def __repr__(self):
        """
        Gets a human-readable (almost) representation of this segment
        """
        return repr(self.__left) + self.operator_name() + repr(self.__right)

    def select(self, root, bca):
        """
        Gets the value for this property path for the given brain root and adapter

        :param root: The root module of the brain
        :param bca: The brain control adapter
        """
        left = self.__left.select(root, bca)
        right = self.__right.select(root, bca)
        return self.operate(left, right)


class NotEqualsSegment(EqualsSegment):
    """
    Represents a segments that checks whether two path elements are not equal

    This is necessary because many builtin classes such as int do not implement __ne__
    """

    def __init__(self, left, right):
        super(NotEqualsSegment, self).__init__(left, right)

    @classmethod
    def operator_name(cls):
        """
        The operator name

        :returns: A string representation of the operator represented by the current element
        """
        return "!="

    @classmethod
    def operate(cls, left, right):
        """
        Actually perform the operator

        :param left: The left operand
        :param right: The right operand
        """
        return left != right


class GreaterThanSegment(EqualsSegment):
    """
    Represents a segments that checks whether a path is greather than another for the current brain

    This is necessary because many builtin classes such as int do not implement __gt__
    """
    def __init__(self, left, right):
        super(GreaterThanSegment, self).__init__(left, right)

    @classmethod
    def operator_name(cls):
        """
        The operator name

        :returns: A string representation of the operator represented by the current element
        """
        return ">"

    @classmethod
    def operate(cls, left, right):
        """
        Actually perform the operator

        :param left: The left operand
        :param right: The right operand
        """
        return left > right


class LessThanSegment(EqualsSegment):
    """
    Represents a segments that checks whether a path is less than another for the current brain

    This is necessary because many builtin classes such as int do not implement __le__
    """
    def __init__(self, left, right):
        super(LessThanSegment, self).__init__(left, right)

    @classmethod
    def operator_name(cls):
        """
        The operator name

        :returns: A string representation of the operator represented by the current element
        """
        return "<"

    @classmethod
    def operate(cls, left, right):
        """
        Actually perform the operator

        :param left: The left operand
        :param right: The right operand
        """
        return left < right


class GreaterThanEqualsSegment(EqualsSegment):
    """
    Represents a segments that checks whether a path is greather than or equals to
    another for the current brain

    This is necessary because many builtin classes such as int do not implement __ge__
    """
    def __init__(self, left, right):
        super(GreaterThanEqualsSegment, self).__init__(left, right)

    @classmethod
    def operator_name(cls):
        """
        The operator name

        :returns: A string representation of the operator represented by the current element
        """
        return ">="

    @classmethod
    def operate(cls, left, right):
        """
        Actually perform the operator

        :param left: The left operand
        :param right: The right operand
        """
        return left >= right


class LessThanEqualsSegment(EqualsSegment):
    """
    Represents a segments that checks whether a path is less than or equals to
    another for the current brain

    This is necessary because many builtin classes such as int do not implement __le__
    """
    def __init__(self, left, right):
        super(LessThanEqualsSegment, self).__init__(left, right)

    @classmethod
    def operator_name(cls):
        """
        The operator name

        :returns: A string representation of the operator represented by the current element
        """
        return "<="

    @classmethod
    def operate(cls, left, right):
        """
        Actually perform the operator

        :param left: The left operand
        :param right: The right operand
        """
        return left <= right


class CustomSegment(PropertyPath):
    """
    This class represents a user-defined path segment
    """

    def __init__(self, fun):
        """
        Creates a new custom path segment for the given function

        :param fun: A function that either accepts one or two arguments
        """
        if callable(fun):
            args = inspect.getargspec(fun).args
            if len(args) == 1:
                self.__fun = lambda r, b: fun(r)
            elif len(args) == 2:
                self.__fun = fun
            elif len(args) == 0:
                self.__fun = lambda r, b: fun()
            else:
                raise Exception("The argument function requires too many parameters. "
                                "You may expect at most two parameters for the loaded "
                                "network module and a brain info object.")
        else:
            raise Exception("The argument of this function should be a function "
                            "accepting one or two parameters.")

    def __repr__(self):
        """
        Gets a human-readable (almost) representation of this segment
        """
        return "(custom)"

    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
        """
        return self.__fun(root, bca)


class IndexPathSegment(PropertyPath):
    """
    Represents an index path segment
    """

    def __init__(self, index, parent):
        """
        Creates a new index path segment for the given index and parent path

        :param index: The index to be used to get to the child element
        :param parent: The parent path
        """
        assert isinstance(parent, PropertyPath)
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

    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
        """
        parent = self.__parent.select(root, bca)
        if parent is None:
            return None
        if bca.is_population(parent):
            return bca.create_view(parent, self.__index)
        else:
            return parent[self.__index]

    @property
    def parent(self):
        """
        Gets the parent node for this element
        """
        return self.__parent


class RangeSegment(PropertyPath):
    """
    Represents a range path segment
    """

    def __init__(self, start, stop, step=None):
        self.__start = RangeSegment._make_path(start)
        self.__stop = RangeSegment._make_path(stop)
        self.__step = RangeSegment._make_path(step)

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return "range({0},{1},step={2})".format(repr(self.__start),
                                                  repr(self.__stop),
                                                  repr(self.__step))

    def select(self, root, bca):
        """
        Selects the path represented by this instance started from the given root object

        :param root: The specified root object
        :param bca: The brain info
        """
        start = self.__start.select(root, bca)
        stop = self.__stop.select(root, bca)
        step = self.__step.select(root, bca)
        return range(start, stop, step or 1)
