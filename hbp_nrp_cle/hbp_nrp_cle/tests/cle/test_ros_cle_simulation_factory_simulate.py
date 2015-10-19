"""
This module tests the simulate routine of the ROSCLESimulationFactory class
"""

__author__ = 'georghinkel'


import unittest
import os
from hbp_nrp_cle.cle.ROSCLESimulationFactory import ROSCLESimulationFactory
import hbp_nrp_cle.tests.cle.dummy_experiment_validation as val

class MockedServiceRequest(object):
    environment_file = "environment_file.sdf"
    generated_cle_script_file = None
    gzserver_host = "local"

class SimulationTestCase(unittest.TestCase):

    def test_simulation(self):

        val.experiment_shutdown_called = False
        val.experiment_cle_init_called = False

        factory = ROSCLESimulationFactory()

        dirpath = os.path.split(__file__)[0]
        MockedServiceRequest.generated_cle_script_file = os.path.join(dirpath, "dummy_experiment.py")

        factory.start_new_simulation(MockedServiceRequest())

        self.assertTrue(val.experiment_cle_init_called)
        self.assertTrue(val.experiment_shutdown_called)
        self.assertEqual(factory.simulation_count, 1)
        self.assertEqual(factory.failed_simulation_count, 0)

    def test_broken_simulation(self):

        val.experiment_shutdown_called = False
        val.experiment_cle_init_called = False

        factory = ROSCLESimulationFactory()

        dirpath = os.path.split(__file__)[0]
        MockedServiceRequest.generated_cle_script_file = os.path.join(dirpath, "dummy_experiment_broken.py")

        self.assertRaises(Exception, factory.start_new_simulation, MockedServiceRequest())

        self.assertTrue(val.experiment_cle_init_called)
        self.assertFalse(val.experiment_shutdown_called)
        self.assertEqual(factory.simulation_count, 1)
        self.assertEqual(factory.failed_simulation_count, 1)