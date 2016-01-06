"""
CLE unit test
"""

from hbp_nrp_cle.cle.ClosedLoopEngine import ClosedLoopEngine
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainControlAdapter, MockBrainCommunicationAdapter
from hbp_nrp_cle.mocks.tf_framework import MockTransferFunctionManager
from geometry_msgs.msg import Point, Pose, Quaternion

import unittest
import threading
from mock import Mock, patch

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
        rcm = MockRobotCommunicationAdapter()
        self.bca = MockBrainControlAdapter()
        bcm = MockBrainCommunicationAdapter()
        tfm = MockTransferFunctionManager()
        self._cle = ClosedLoopEngine(rca, rcm, self.bca, bcm, tfm, 0.01)

    def test_run_step(self):
        """
        Test a single simulation step.
        """
        self._cle.initialize("foo")
        self.assertEqual(self._cle.run_step(0.01), 0.01)

    def test_get_time(self):
        """
        Test if the simulation run for the right time.
        """
        self._cle.initialize("foo")
        self._cle.run_step(0.05)
        self._cle.wait_step()
        self.assertEqual(self._cle.simulation_time, 0.05)

    def test_start_stop(self):
        """
        Test start, pause and restart.
        """
        self._cle.initialize("foo")

        def stopcle(*args, **kwargs):
            args[0].stop()

        t = threading.Timer(2.0, stopcle, [self._cle])
        t.start()
        self._cle.start()
        self.assertGreater(self._cle.real_time, 0.0)

    def test_reset(self):
        self._cle.initialize("foo")
        self._cle.run_step(0.05)
        self._cle.wait_step()
        self._cle.reset()
        self.assertEqual(self._cle.simulation_time, 0.0)
        self.assertEqual(self._cle.real_time, 0.0)

    def test_shutdown(self):
        """
        Test shutdown.
        """
        self._cle.initialize("foo")
        self._cle.shutdown()

    @patch('hbp_nrp_cle.cle.ClosedLoopEngine.set_model_pose')
    def test_reset_robot_pose(self, set_model_pose_mock):
        pose = Pose()
        pose.position = Point(0, 0, 0)
        pose.orientation = Quaternion(0, 0, 0, 1)

        self._cle.initial_robot_pose = pose
        self._cle.reset_robot_pose()
        set_model_pose_mock.assert_called_with('robot', pose)

if __name__ == '__main__':
    unittest.main()
