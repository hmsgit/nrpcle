"""
Unit tests for the ROSCLESimulationFactoryClient
"""
from mock import patch, Mock
from hbp_nrp_cle.cle.ROSCLESimulationFactoryClient import ROSCLESimulationFactoryClient

__author__ = 'Alessandro Ambrosano'

import unittest


class TestROSCLESimulationFactoryClient(unittest.TestCase):

    @patch('hbp_nrp_cle.cle.ROSCLESimulationFactoryClient.rospy.ServiceProxy')
    def test_start_new_simulation(self, mock_service_proxy):
        roscle = ROSCLESimulationFactoryClient()
        m = Mock()
        m.success = True
        m.status_message = ''
        roscle._ROSCLESimulationFactoryClient__start_new_simulation_service = Mock(return_value=m)
        self.assertIsNone(roscle.start_new_simulation(None, None))
        m.success = False
        self.assertRaises(roscle.start_new_simulation)


if __name__ == '__main__':
    unittest.main()