"""
Tests for the PropertyPath module
"""

__author__ = 'GeorgHinkel'

import unittest
from hbp_nrp_cle.tf_framework._PropertyPath import PropertyPath


class Dummy(object):
    """
    A simple dummy object without functionality
    """

    def __init__(self, item):
        """
        Creates a new dummy instance
        """
        self.item = item


class PropertyPathTests(unittest.TestCase):
    """
    Tests for the Property Path module
    """

    def setUp(self):
        self.dummy = Dummy([0, 8, 15])

    def test_root(self):
        root = PropertyPath()

        assert root.__repr__() == "(root)"
        assert root.select(self.dummy) is self.dummy

    def test_item(self):
        root = PropertyPath()
        item = root.item

        assert item.__repr__() == "(root).item"
        assert item.select(self.dummy) is self.dummy.item

    def test_index_slice(self):
        root = PropertyPath()
        index = root[0:2]

        assert index.__repr__() == "(root)[slice(0, 2, None)]"
        assert index.select(self.dummy.item) == self.dummy.item[0:2]

    def test_index_int(self):
        root = PropertyPath()
        index = root[0]

        assert index.__repr__() == "(root)[slice(0, 1, None)]"
        assert index.select(self.dummy.item) == self.dummy.item[0:1]
