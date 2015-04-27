"""
Tests the neuron selectors
"""

__author__ = 'GeorgHinkel'

import unittest
from hbp_nrp_cle.tests.tf_framework.test_property_path import Dummy
import hbp_nrp_cle.tf_framework as nrp


class NeuronSelectorTests(unittest.TestCase):
    """
    Tests for the Property Path module
    """

    def setUp(self):
        self.dummy = Dummy([0, 8, 15])

    def test_mapping(self):
        root = nrp.brain
        mapping = nrp.map_neurons(range(1, 4), lambda i: root.item[i])

        assert mapping.__repr__().startswith("mapping ")
        result = mapping.select(self.dummy)
        self.assertEqual(2, len(result))
        self.assertEqual(8, result[0])
        self.assertEqual(15, result[1])

    def test_embedded_mapping(self):
        root = nrp.brain
        mapping = nrp.map_neurons(range(1,2),
                                  lambda x: nrp.map_neurons(range(1, 4), lambda i: root.item[i]))

        assert mapping.__repr__().startswith("mapping ")
        result = mapping.select(self.dummy)
        self.assertEqual(2, len(result))
        self.assertEqual(8, result[0])
        self.assertEqual(15, result[1])

    def test_chain(self):
        root = nrp.brain
        chain = nrp.chain_neurons(nrp.map_neurons(range(0,2), lambda i: root.item[i]), root.item[2])

        result = chain.select(self.dummy)
        self.assertEqual(3, len(result))
        self.assertEqual(0, result[0])
        self.assertEqual(8, result[1])
        self.assertEqual(15, result[2])

    def test_simplified_chain(self):
        root = nrp.brain
        chain = nrp.chain_neurons(root.item[1], root.item[2])

        self.assertEqual("[(root).item[slice(1, 2, None)],(root).item[slice(2, 3, None)]]",
                         chain.__repr__())
        result = chain.select(self.dummy)
        self.assertEqual(2, len(result))
        self.assertEqual(8, result[0])
        self.assertEqual(15, result[1])

if __name__ == "__main__":
    unittest.main()
