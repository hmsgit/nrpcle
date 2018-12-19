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
brain_loader unit test
"""

from hbp_nrp_cle.brainsim.common import PythonBrainLoader as BrainLoader
from hbp_nrp_cle.brainsim.pynn import H5PyNNBrainLoader as H5BrainLoader
import hbp_nrp_cle.tf_framework as nrp

import unittest
import os
import pyNN.nest as sim
from mock import patch, Mock
__author__ = 'Lorenzo Vannucci'


# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase

MockOs = Mock()
MockOs.environ = {'NRP_SIMULATION_DIR': '/somewhere/near/the/rainbow'}
@patch("hbp_nrp_cle.common.os", new=MockOs)
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
        module = H5BrainLoader.load_h5_network(filename, sim, **{'sensors': [0, 1, 2], 'actors': [3, 4, 5]})
        self.assertIsInstance(module.circuit, sim.Population)

    def test_load_python_network(self):
        """
        Tests loading a Python brain model
        """
        self.assertEquals(nrp.config.brain_source, None)
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModel.py')
        BrainLoader.is_brain_safely_imported = Mock(return_value=True)
        module = BrainLoader.load_py_network(filename)

        circuit = module.circuit
        self.assertIsInstance(circuit, sim.Population)
        self.assertEqual(3, len(circuit))

        BrainLoader.setup_access_to_population(module, **{'first': slice(0, 1), 'second': slice(1, 3)})
        first = module.first
        second = module.second
        self.assertIsInstance(first, sim.PopulationView)
        self.assertIsInstance(second, sim.PopulationView)
        self.assertEqual(1, len(first))
        self.assertEqual(2, len(second))

    def test_load_population_slice_out_of_bounds(self):
        """
        Tests loading a population with a slice out of bounds
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModel.py')
        module = BrainLoader.load_py_network(filename)
        self.assertRaises(Exception, BrainLoader.setup_access_to_population, module, first=slice(0, 4))

    def test_load_python_network_exception(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModelNoCircuit.py')
        module = BrainLoader.load_py_network(filename)
        self.assertRaises(Exception, BrainLoader.setup_access_to_population, module, first=slice(0, 1))

    def test_load_python_network_no_extra_population(self):
        """
        Tests loading a Python brain model
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModelNoCircuit.py')
        BrainLoader.is_brain_safely_imported = Mock(return_value=True)
        module = BrainLoader.load_py_network(filename)
        self.assertIsInstance(module.foo, sim.Population)
        self.assertEqual(3, len(module.foo))

    @patch("hbp_nrp_cle.common.refresh_resources")
    def test_setup_populations(self, refresh_resources_mock):
        """
        Tests setting up additional populations
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'DummyBrainModel.py')
        module = BrainLoader.load_py_network(filename)
        self.assertEqual(module.circuit.size, 3)
        self.assertIsInstance(module.circuit, sim.Population)

        add_pop = {'testPopulation1': slice(0, 1, 1),
                   'testPopulation2': slice(1, 2, 1)}
        BrainLoader.setup_access_to_population(module, **add_pop)
        self.assertIsNotNone(module.testPopulation1)
        self.assertEquals(module.testPopulation1[0], module.circuit[0])
        self.assertIsNotNone(module.testPopulation2)
        self.assertEquals(module.testPopulation2[0], module.circuit[1])


if __name__ == '__main__':
    unittest.main()
