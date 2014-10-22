"""
This module contains a selector class selecting property paths
"""

__author__ = 'GeorgHinkel'


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


class AttributePathSegment(PropertyPath):
    """
    Represents an attribute segment in a property path
    """

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

    def __repr__(self):
        """
        Gets a string representation of the path
        """
        return self.__parent.__repr__() + "." + self.__name

    def select(self, root):
        """
        Selects the path represented by this instance started from the given root object
        :param root: The specified root object
        """
        parent = self.__parent.select(root)
        if parent is None:
            return None
        return getattr(parent, self.__name)


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
