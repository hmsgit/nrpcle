"""
brain_loader unit test
"""

from hbp_nrp_cle.brainsim.pynn import PyNNBrainLoader as BrainLoader
import hbp_nrp_cle.tf_framework as nrp

import unittest
import os

import pyNN.nest as sim

__author__ = 'Lorenzo Vannucci'


# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase
class TestClosedLoopEngine(unittest.TestCase):
    """
    Tests the brain_loader utility
    """

    # pylint: disable=R0201
    # method has to be a method in order to be run as part of the test suite
    def test_load_h5_network(self):
        """
        Test loading an .h5 file.
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'braitenberg.h5')
        BrainLoader.load_h5_network(filename, {'sensors': [0, 1, 2], 'actors': [3, 4, 5]})

    def test_load_python_network(self):
        """
        Tests loading a Python brain model
        """
        self.assertEquals(nrp.config.brain_source, None)
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModel.py')
        BrainLoader.load_py_network(filename, {'first': slice(0, 1), 'second': slice(1, 3)})
        first = nrp.config.brain_root.first
        second = nrp.config.brain_root.second
        circuit = nrp.config.brain_root.circuit
        self.assertIsInstance(circuit, sim.Population)
        self.assertIsInstance(first, sim.PopulationView)
        self.assertIsInstance(second, sim.PopulationView)
        self.assertEqual(3, len(circuit))
        self.assertEqual(1, len(first))
        self.assertEqual(2, len(second))

    def test_load_python_network_exception(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModelNoCircuit.py')
        self.assertRaises(Exception, BrainLoader.load_py_network, filename, {'first': slice(0, 1)})

    def test_load_python_network_no_extra_population(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModelNoCircuit.py')
        BrainLoader.load_py_network(filename, {})
        foo = nrp.config.brain_root.foo
        self.assertIsInstance(foo, sim.Population)
        self.assertEqual(3, len(foo))

if __name__ == '__main__':
    unittest.main()
