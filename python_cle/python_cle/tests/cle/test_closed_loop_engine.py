"""
CLE unit test
"""

from python_cle.cle.ClosedLoopEngine import ClosedLoopEngine
from python_cle.mocks.robotsim.MockRobotControlAdapter import (
    MockRobotControlAdapter
)
from python_cle.mocks.brainsim.MockBrainControlAdapter import (
    MockBrainControlAdapter
)
from python_cle.mocks.tf_framework.MockTransferFunctionManager import (
    MockTransferFunctionManager
)

import unittest
import time

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
        rca = MockRobotControlAdapter()
        bca = MockBrainControlAdapter()
        tfm = MockTransferFunctionManager()
        self._cle = ClosedLoopEngine(rca, bca, tfm, 0.01)
        self._cle.initialize()

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
        self.assertEqual(self._cle.time, 0.05)

    def test_start_stop(self):
        """
        Test start, pause and restart.
        """
        self._cle.start()
        time.sleep(2)
        self._cle.stop()
        time.sleep(1)
        self._cle.start()
        time.sleep(2)
        self._cle.stop()
        time.sleep(1)

    def test_shutdown(self):
        """
        Test shutdown.
        """
        self._cle.shutdown()


if __name__ == '__main__':
    unittest.main()
