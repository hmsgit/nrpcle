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

__author__ = 'Georg Hinkel'

import unittest
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter


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
        self.bca = MockBrainCommunicationAdapter()

    def test_root(self):
        root = nrp.brain

        self.assertEqual("(root)", root.__repr__())
        self.assertIs(self.dummy, root.select(self.dummy, self.bca))

    def test_item(self):
        root = nrp.brain
        item = root.item

        self.assertEqual("(root).item", item.__repr__())
        self.assertIs(self.dummy.item, item.select(self.dummy, self.bca))

    def test_index_slice(self):
        root = nrp.brain
        index = root[0:2]

        self.assertEqual("(root)[slice(0, 2, None)]", index.__repr__())
        self.assertEqual(self.dummy.item[0:2], index.select(self.dummy.item, self.bca))

    def test_index_int(self):
        root = nrp.brain
        index = root[0]

        self.bca.create_view = lambda p, sl: p[sl:(sl+1)]

        self.assertEqual("(root)[0]", index.__repr__())
        self.assertEqual(self.dummy.item[0:1], index.select(self.dummy.item, self.bca))

    def test_sum_path_path(self):
        root = nrp.brain
        sum = root.item[0] + root.item[1]

        self.assertEqual("(root).item[0].__add__((root).item[1])", repr(sum))
        self.assertEqual(8, sum.select(self.dummy, self.bca))

    def test_sum_path_val(self):
        root = nrp.brain
        sum = root.item[0] + 1

        self.assertEqual("(root).item[0].__add__(1)", repr(sum))
        self.assertEqual(1, sum.select(self.dummy, self.bca))

    def test_sum_val_path(self):
        root = nrp.brain
        sum = 1 + root.item[0]

        self.assertEqual("1.__add__((root).item[0])", repr(sum))
        self.assertEqual(1, sum.select(self.dummy, self.bca))

    def test_mul_path_path(self):
        root = nrp.brain
        prod = root.item[1] * root.item[2]

        self.assertEqual("(root).item[1].__mul__((root).item[2])", repr(prod))
        self.assertEqual(120, prod.select(self.dummy, self.bca))

    def test_mul_path_val(self):
        root = nrp.brain
        prod = root.item[1] * 2

        self.assertEqual("(root).item[1].__mul__(2)", repr(prod))
        self.assertEqual(16, prod.select(self.dummy, self.bca))

    def test_mul_val_path(self):
        root = nrp.brain
        prod = 2 * root.item[1]

        self.assertEqual("2.__mul__((root).item[1])", repr(prod))
        self.assertEqual(16, prod.select(self.dummy, self.bca))

    def test_sub_path_path(self):
        root = nrp.brain
        diff = root.item[1] - root.item[0]

        self.assertEqual("(root).item[1].__sub__((root).item[0])", repr(diff))
        self.assertEqual(8, diff.select(self.dummy, self.bca))

    def test_sub_path_val(self):
        root = nrp.brain
        diff = root.item[1] - 1

        self.assertEqual("(root).item[1].__sub__(1)", repr(diff))
        self.assertEqual(7, diff.select(self.dummy, self.bca))

    def test_sub_val_path(self):
        root = nrp.brain
        diff = 1 - root.item[1]

        self.assertEqual("1.__sub__((root).item[1])", repr(diff))
        self.assertEqual(-7, diff.select(self.dummy, self.bca))

    def test_div_path_path(self):
        root = nrp.brain
        div = root.item[2] / root.item[1]

        self.assertEqual("(root).item[2].__div__((root).item[1])", repr(div))
        self.assertEqual(15 / 8, div.select(self.dummy, self.bca))

    def test_div_path_val(self):
        root = nrp.brain
        div = root.item[1] / 2

        self.assertEqual("(root).item[1].__div__(2)", repr(div))
        self.assertEqual(4, div.select(self.dummy, self.bca))

    def test_div_val_path(self):
        root = nrp.brain
        div = 16 / root.item[1]

        self.assertEqual("16.__div__((root).item[1])", repr(div))
        self.assertEqual(2, div.select(self.dummy, self.bca))

    def test_eq_val(self):
        root = nrp.brain
        equals = root.item[1] == 8

        self.assertEqual("(root).item[1]==8", repr(equals))
        self.assertTrue(equals.select(self.dummy, self.bca))

    def test_eq_path(self):
        root = nrp.brain
        equals = root.item[1] == root.item[0]

        self.assertEqual("(root).item[1]==(root).item[0]", repr(equals))
        self.assertFalse(equals.select(self.dummy, self.bca))

    def test_ne_val(self):
        root = nrp.brain
        equals = root.item[1] != 8

        self.assertEqual("(root).item[1]!=8", repr(equals))
        self.assertFalse(equals.select(self.dummy, self.bca))

    def test_ne_path(self):
        root = nrp.brain
        equals = root.item[1] != root.item[0]

        self.assertEqual("(root).item[1]!=(root).item[0]", repr(equals))
        self.assertTrue(equals.select(self.dummy, self.bca))

    def test_and_path_val(self):
        root = nrp.brain
        _and = root.item[1] & 5

        self.assertEqual("(root).item[1].__and__(5)", repr(_and))
        self.assertEqual(0, _and.select(self.dummy, self.bca))

    def test_and_val_path(self):
        root = nrp.brain
        _and = 5 & root.item[1]

        self.assertEqual("5.__and__((root).item[1])", repr(_and))
        self.assertEqual(0, _and.select(self.dummy, self.bca))

    def test_and_path_path(self):
        root = nrp.brain
        _and = root.item[1] & root.item[2]

        self.assertEqual("(root).item[1].__and__((root).item[2])", repr(_and))
        self.assertEqual(8, _and.select(self.dummy, self.bca))

    def test_or_path_val(self):
        root = nrp.brain
        _and = root.item[1] | 5

        self.assertEqual("(root).item[1].__or__(5)", repr(_and))
        self.assertEqual(13, _and.select(self.dummy, self.bca))

    def test_or_val_path(self):
        root = nrp.brain
        _and = 5 | root.item[1]

        self.assertEqual("5.__or__((root).item[1])", repr(_and))
        self.assertEqual(13, _and.select(self.dummy, self.bca))

    def test_or_path_path(self):
        root = nrp.brain
        _and = root.item[1] | root.item[2]

        self.assertEqual("(root).item[1].__or__((root).item[2])", repr(_and))
        self.assertEqual(15, _and.select(self.dummy, self.bca))

    def test_xor_path_val(self):
        root = nrp.brain
        _and = root.item[1] ^ 5

        self.assertEqual("(root).item[1].__xor__(5)", repr(_and))
        self.assertEqual(13, _and.select(self.dummy, self.bca))

    def test_xor_val_path(self):
        root = nrp.brain
        _and = 5 ^ root.item[1]

        self.assertEqual("5.__xor__((root).item[1])", repr(_and))
        self.assertEqual(13, _and.select(self.dummy, self.bca))

    def test_xor_path_path(self):
        root = nrp.brain
        _and = root.item[1] ^ root.item[2]

        self.assertEqual("(root).item[1].__xor__((root).item[2])", repr(_and))
        self.assertEqual(7, _and.select(self.dummy, self.bca))

    def test_range(self):
        root = nrp.brain
        nrange1 = nrp.nrange(0, 15, 8)
        nrange2 = nrp.nrange(0, root.item[2], root.item[1])

        self.assertEqual("range(0,15,step=8)", repr(nrange1))
        self.assertEqual("range(0,(root).item[2],step=(root).item[1])", repr(nrange2))
        self.assertEqual(range(0, 15, 8), nrange1.select(self.dummy, self.bca))
        self.assertEqual(range(0, 15, 8), nrange2.select(self.dummy, self.bca))

    def test_resolve(self):
        resolved = nrp.resolve(lambda root: root.item[2])

        self.assertEqual("(custom)", repr(resolved))
        self.assertEqual(15, resolved.select(self.dummy, self.bca))

        resolved2 = nrp.resolve(lambda root, bca: root.item[1])
        self.assertEqual("(custom)", repr(resolved2))
        self.assertEqual(8, resolved2.select(self.dummy, self.bca))

        self.assertRaises(Exception, nrp.resolve, 0)
        self.assertRaises(Exception, nrp.resolve, lambda a,b,c: None)


if __name__ == "__main__":
    unittest.main()
