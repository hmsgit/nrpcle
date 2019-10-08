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

import unittest
from mock import Mock, patch, mock_open
import nest
from hbp_nrp_cle.brainsim.nest.NestControlAdapter import NestControlAdapter
from hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter import NestCommunicationAdapter
import hbp_nrp_cle.tf_framework as tf_framework
from hbp_nrp_cle.cle.CLEInterface import BrainRuntimeException
import os
from testfixtures import log_capture

class NestControlAdapterTest(unittest.TestCase):

    def setUp(self):
        """
        Initialize the Nest adapters
        """

        self.sim_state = Mock()
        self.sim_state.load_brain = Mock()
        self.sim_state.initialize = Mock()

        self.control = NestControlAdapter(self.sim_state)
        self.communicator = NestCommunicationAdapter()

    @patch('hbp_nrp_cle.brainsim.nest.NestControlAdapter.setup_access_to_population')
    @patch('hbp_nrp_cle.brainsim.nest.NestControlAdapter.BrainLoader')
    def test_load_brain(self, loader, access_to_population):

        with patch('hbp_nrp_cle.brainsim.nest.NestControlAdapter.open',
                   mock_open(read_data='some python code'),
                   create=True):

            slice1 = {'from': 1, 'to': 2, 'step': 3}
            slice2 = {'from': 4, 'to': 5, 'step': None}

            populations_mixed = {
                'slice_1': slice1, 'slice_2': slice(4, 5),
                'list_1': [1, 2, 3]
            }
            self.control.load_brain("filename.py", **populations_mixed)

            loader.load_py_network.assert_called_with("filename.py")
            access_to_population.assert_called_once()
            self.assertTrue(loader.load_py_network.called)
            self.assertEqual(tf_framework.config.brain_source, 'some python code')

            populations_json = {
                'slice_1': slice1, 'slice_2': slice2,
                'list_1': [1, 2, 3]
            }
            self.assertEqual(
                tf_framework.config.brain_populations,
                populations_json
            )
            loader.load_py_network.reset_mock()

    @patch('hbp_nrp_cle.brainsim.nest.NestControlAdapter.BrainLoader')
    def test_load_brain_no_population(self, _):

        with patch('hbp_nrp_cle.brainsim.nest.NestControlAdapter.open',
                   mock_open(read_data='some python code'),
                   create=True):

            current_populations = {
                'slice_1': {'from': 1, 'to': 2, 'step': 3}
            }

            brain_pop_bak = tf_framework.config.brain_populations
            tf_framework.config.brain_populations = current_populations

            with patch.object(self.control, 'load_populations') as load_pops_patch:
                self.control.load_brain("foo.py")
                load_pops_patch.assert_called_with(**current_populations)

            tf_framework.config.brain_populations = brain_pop_bak

    def test_populations_no_brain_loaded(self):

        populations = {'slice_1': {'from': 1, 'to': 2, 'step': 3}}

        brain_root_bak = tf_framework.config.brain_root
        tf_framework.config.brain_root = None

        # brain_root is None
        with self.assertRaises(Exception):
            self.control.load_populations(**populations)

        # brain_root.circuit missing
        tf_framework.config.brain_root = Mock(spec=[])

        with self.assertRaises(AttributeError):
            self.control.load_populations(**populations)

        tf_framework.config.brain_root = brain_root_bak

    def test_populations_using_json_slice(self):
        slice1 = {'from': 1, 'to': 2, 'step': 3}
        slice2 = {'from': 1, 'to': 2, 'step': None}
        populations_json_slice = {
          'slice_1': slice1, 'slice_2': slice2,
          'list_1': [1, 2, 3]
        }
        populations_python_slice = {
          'slice_1': slice(1, 2, 3), 'slice_2': slice(1, 2),
          'list_1': [1, 2, 3]
        }
        self.assertEqual(
          self.control.populations_using_json_slice(populations_json_slice),
          populations_json_slice
        )

        self.assertEqual(
          self.control.populations_using_json_slice(populations_python_slice),
          populations_json_slice
        )

    def test_populations_using_python_slice(self):
        slice1 = {'from': 1, 'to': 2, 'step': 3}
        slice2 = {'from': 1, 'to': 2}
        populations_json_slice = {
          'population_1': 1, 'population_2': 2,
          'slice_1': slice1, 'slice_2': slice2,
          'list_1': [1, 2, 3]
        }
        populations_python_slice = {
          'population_1': 1, 'population_2': 2,
          'slice_1': slice(1, 2, 3), 'slice_2': slice(1, 2),
          'list_1': [1, 2, 3]
        }
        self.assertEqual(
          self.control.populations_using_python_slice(populations_json_slice),
          populations_python_slice
        )

        self.assertEqual(
          self.control.populations_using_python_slice(populations_python_slice),
          populations_python_slice
        )

    def test_shutdown(self):

        self.control.shutdown()

        # communicator's lists must be empty
        self.assertEquals(len(self.communicator.detector_devices), 0)
        self.assertEquals(len(self.communicator.refreshable_devices), 0)
        self.assertEquals(len(self.communicator.finalizable_devices), 0)

        self.assertFalse(self.control.is_initialized)
        self.assertFalse(self.communicator.is_initialized)

    def test_exception_is_raised(self):
        sim = Mock()
        sim.Simulate.side_effect = Exception("error")
        adapter = NestControlAdapter(sim)
        with self.assertRaises(BrainRuntimeException):
            adapter.run_step(1)

    def test_populations(self):
        tf_framework.config.brain_root = None
        tf_framework.config.brain_populations = {}
        adapter = NestControlAdapter(nest)
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'example_nest_brain.py')
        adapter.is_brain_safely_imported = Mock(return_value=True)
        adapter.load_brain(filename)
        populations = adapter.get_populations()
        self.assertEqual(3, len(populations))
        circuit = next(p for p in populations if p.name == "circuit")
        pop1 = next(p for p in populations if p.name == "pop1")
        pop2 = next(p for p in populations if p.name == "pop2")
        # check circuit
        self.assertEqual("circuit", circuit.name)
        self.assertEqual("iaf_psc_alpha", circuit.celltype)
        self.assertNotEqual(0, len(circuit.parameters))
        self.assertEqual(3, len(circuit.gids))
        self.assertEqual(2, circuit.indices[2])
        # check pop1
        self.assertEqual("pop1", pop1.name)
        self.assertEqual("iaf_psc_alpha", pop1.celltype)
        self.assertNotEqual(0, len(pop1.parameters))
        self.assertEqual(2, len(pop1.gids))
        self.assertEqual(1, pop1.indices[1])
        # check pop2
        self.assertEqual("pop2", pop2.name)
        self.assertEqual("iaf_psc_alpha", pop2.celltype)
        self.assertNotEqual(0, len(pop2.parameters))
        self.assertEqual(2, len(pop2.gids))
        self.assertEqual(1, pop2.indices[1])

    @log_capture('hbp_nrp_cle.brainsim.nest.NestControlAdapter')
    def test_reset(self, logcapture):
        """
        Test the reset functionality. The reset must not unload the brain, therefore the neurons
        that have been setup before must still exist after the reset.
        """
        nest.ResetKernel()
        nest.Create('iaf_psc_exp', 10)
        self.control.reset()

        population = nest.Create('iaf_psc_exp', 10)
        self.assertEqual(population[9], 20)
        logcapture.check(('hbp_nrp_cle.brainsim.nest.NestControlAdapter', 'INFO',
                          'neuronal simulator reset'))

    def tearDown(self):
        """
        Shuts down the Brain control
        """
        self.control.shutdown()
        tf_framework.config.brain_root = None
        tf_framework.config.brain_populations = {}


if __name__ == "__main__":
    unittest.main()
