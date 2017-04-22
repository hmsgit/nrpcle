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
Tests for the PropertyPath module
"""

__author__ = 'GeorgHinkel'

import unittest
import hbp_nrp_cle.tf_framework as nrp


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
        root = nrp.brain

        self.assertEqual("(root)", root.__repr__())
        self.assertIs(self.dummy, root.select(self.dummy))

    def test_item(self):
        root = nrp.brain
        item = root.item

        self.assertEqual("(root).item", item.__repr__())
        self.assertIs(self.dummy.item, item.select(self.dummy))

    def test_index_slice(self):
        root = nrp.brain
        index = root[0:2]

        self.assertEqual("(root)[slice(0, 2, None)]", index.__repr__())
        self.assertEqual(self.dummy.item[0:2], index.select(self.dummy.item))

    def test_index_int(self):
        root = nrp.brain
        index = root[0]

        self.assertEqual("(root)[slice(0, 1, None)]", index.__repr__())
        self.assertEqual(self.dummy.item[0:1], index.select(self.dummy.item))


if __name__ == "__main__":
    unittest.main()
