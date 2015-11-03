"""
CLE unit test
"""

from hbp_nrp_cle.cle.ClosedLoopEngine import ClosedLoopEngine
from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter, MockRobotControlAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter, MockBrainControlAdapter
from hbp_nrp_cle.mocks.tf_framework import MockTransferFunctionManager

import unittest
import time
from testfixtures import log_capture, LogCapture
from mock import Mock

__author__ = 'Nino Cauli'


# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase
class TestClosedLoopEngine(unittest.TestCase):
    """
    Tests the closed loop engine
    """

    def setUp(self):
        """
        Sets up the cle and the mocks for the adapters.
        """
        with LogCapture('hbp_nrp_cle.cle.ClosedLoopEngine') as l:
            rca = MockRobotControlAdapter()
            rcm = MockRobotCommunicationAdapter()
            self.bca = MockBrainControlAdapter()
            bcm = MockBrainCommunicationAdapter()
            self.tfm = MockTransferFunctionManager()
            self._cle = ClosedLoopEngine(rca, rcm, self.bca, bcm, self.tfm, 0.01)
            self._cle.initialize()
        l.check(('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                 'robot control adapter ready'),
                ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                 'brain control adapter ready'),
                ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                 'transfer function ready'),
                ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                 'CLE started'),
                ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                 'CLE initialized'))

    def test_run_step(self):
        """
        Test a single simulation step.
        """
        self.assertEqual(self._cle.run_step(0.01), 0.01)

    def test_get_time(self):
        """
        Test if the simulation run for the right time.
        """
        self._cle.run_step(0.05)
        self._cle.wait_step()
        self.assertEqual(self._cle.simulation_time, 0.05)

    @log_capture('hbp_nrp_cle.cle.ClosedLoopEngine')
    def test_start_stop(self, logcapture):
        """
        Test start, pause and restart.
        """
        self._cle.start()
        time.sleep(2)
        self.assertGreater(self._cle.real_time, 0.0)
        self._cle.stop()
        time.sleep(1)
        self._cle.start()
        time.sleep(2)
        self._cle.stop()
        time.sleep(1)
        logcapture.check(('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'simulation started'),
                         ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'simulation stopped'),
                         ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'simulation started'),
                         ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'simulation stopped'))

    @log_capture('hbp_nrp_cle.cle.ClosedLoopEngine')
    def test_reset(self, logcapture):
        self._cle.run_step(0.05)
        self._cle.wait_step()
        self._cle.reset()
        self.assertEqual(self._cle.simulation_time, 0.0)
        self.assertEqual(self._cle.real_time, 0.0)
        logcapture.check(('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'simulation stopped'),
                         ('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'CLE reset'))

    @log_capture('hbp_nrp_cle.cle.ClosedLoopEngine')
    def test_shutdown(self, logcapture):
        """
        Test shutdown.
        """
        self._cle.shutdown()
        logcapture.check(('hbp_nrp_cle.cle.ClosedLoopEngine', 'INFO',
                          'simulations shutdown'))

    def test_load_brain(self):
        shutdown_mock = Mock()
        reset_mock = Mock()
        self.bca.shutdown = shutdown_mock
        self.tfm.hard_reset_brain_devices = reset_mock
        self._cle.load_brain("foo.py")
        self.assertTrue(shutdown_mock.called)
        self.assertTrue(reset_mock.called)
        self._cle.load_brain("foo2.py")
        self.assertEqual(2, shutdown_mock.call_count)
        self.assertEqual(2, reset_mock.call_count)
        self._cle.network_file = "foo3.py"
        self.assertEqual(3, shutdown_mock.call_count)
        self.assertEqual(3, reset_mock.call_count)
        self._cle.network_configuration = {"name" : "foo"}
        self.assertEqual(4, shutdown_mock.call_count)
        self.assertEqual(4, reset_mock.call_count)

if __name__ == '__main__':
    unittest.main()
