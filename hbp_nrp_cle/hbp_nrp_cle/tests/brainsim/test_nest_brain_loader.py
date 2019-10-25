# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
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
NEST brain_loader unit test
"""

from hbp_nrp_cle.brainsim.common import PythonBrainLoader as BrainLoader
from hbp_nrp_cle.brainsim.nest.NestControlAdapter import setup_access_to_population

import unittest
import os
from mock import patch, Mock

# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase

__author__ = 'LorenzoVannucci'


class TestNestBrainLoader(unittest.TestCase):
    """
    Tests the NEST brain_loader utility
    """

    # pylint: disable=R0201
    # method has to be a method in order to be run as part of the test suite
    def test_load_python_network(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nest_brain.py')
        BrainLoader.is_brain_safely_imported = Mock(return_value=True)
        module = BrainLoader.load_py_network(filename)

        circuit = module.circuit
        self.assertIsInstance(circuit, tuple)
        self.assertEqual(3, len(circuit))
        module.populations_keys = []
        setup_access_to_population(module, **{'first': slice(0, 1), 'second': slice(1, 3)})
        first = module.first
        second = module.second
        self.assertIsInstance(first, tuple)
        self.assertIsInstance(second, tuple)
        self.assertEqual(1, len(first))
        self.assertEqual(2, len(second))

    def test_load_population_slice_out_of_bounds(self):
        """
        Tests loading a population with a slice out of bounds
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nest_brain.py')
        module = BrainLoader.load_py_network(filename)
        self.assertRaises(Exception, setup_access_to_population, module, first=slice(0, 4))

    def test_load_python_network_exception(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nest_brain_nocircuit.py')
        module = BrainLoader.load_py_network(filename)
        self.assertRaises(Exception, setup_access_to_population, module, first=slice(0, 1))

    def test_load_python_network_no_extra_population(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nest_brain_nocircuit.py')
        BrainLoader.is_brain_safely_imported = Mock(return_value=True)
        module = BrainLoader.load_py_network(filename)

        self.assertIsInstance(module.foo, tuple)
        self.assertEqual(3, len(module.foo))

    def test_setup_populations(self):  # , refresh_resources_mock):
        """
        Tests setting up additional populations
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nest_brain.py')
        module = BrainLoader.load_py_network(filename)
        self.assertEqual(len(module.circuit), 3)
        self.assertIsInstance(module.circuit, tuple)

        test_pop_names = ['testPopulation0', 'testPopulation1']
        test_slices = [slice(0, 1, 1), slice(1, 2, 1)]

        add_pop = dict(zip(test_pop_names, test_slices))

        module.populations_keys = []
        setup_access_to_population(module, **add_pop)

        for i, test_pop_name in enumerate(test_pop_names):
            test_population = getattr(module, test_pop_name)
            self.assertIsNotNone(test_population)
            self.assertEquals(test_population[0], module.circuit[i])


if __name__ == '__main__':
    unittest.main()
