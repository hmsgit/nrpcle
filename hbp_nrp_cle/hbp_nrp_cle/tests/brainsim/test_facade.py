"""
Tests the brainsim facade
"""

__author__ = "Sebastian Krach"
import unittest
import mock
import hbp_nrp_cle.brainsim
import hbp_nrp_cle.brainsim.config

class TestBrainAdapterFacade(unittest.TestCase):
    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_COMMUNICATION_ADAPTER")
    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_CONTROL_ADAPTER")
    def test_control_adapter(self, cont_mock, com_mock):
        cont_mock.return_value = "CONTROL_TYPE"
        com_mock.return_value = "COMMUNICATION_TYPE"

        hbp_nrp_cle.brainsim.config.communication_adapter_type = None
        hbp_nrp_cle.brainsim.config.control_adapter_type = None

        adapter = hbp_nrp_cle.brainsim.instantiate_control_adapter()
        self.assertEqual(adapter, "CONTROL_TYPE")

        with mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_CONTROL_ADAPTER") as new_cont:
            new_cont.return_value = "SHOULD_NOT_PROPAGATE"
            adapter = hbp_nrp_cle.brainsim.instantiate_control_adapter()
            self.assertEqual(adapter, "CONTROL_TYPE")

        adapter = hbp_nrp_cle.brainsim.instantiate_communication_adapter()
        self.assertEqual(adapter, "COMMUNICATION_TYPE")


    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_COMMUNICATION_ADAPTER")
    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_CONTROL_ADAPTER")
    def test_communication_adapter(self, cont_mock, com_mock):
        cont_mock.return_value = "CONTROL_TYPE"
        com_mock.return_value = "COMMUNICATION_TYPE"

        hbp_nrp_cle.brainsim.config.communication_adapter_type = None
        hbp_nrp_cle.brainsim.config.control_adapter_type = None

        adapter = hbp_nrp_cle.brainsim.instantiate_communication_adapter()
        self.assertEqual(adapter, "COMMUNICATION_TYPE")


