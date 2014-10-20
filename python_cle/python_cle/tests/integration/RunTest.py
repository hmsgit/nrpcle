"""
Execute integration test as a unit test
"""

import unittest
import time
# pylint: disable=W0401
from python_cle.tests.integration import MS2TransferFunctions
from python_cle.brainsim import PyNNCommunicationAdapter, PyNNControlAdapter
from python_cle.robotsim import RosCommunicationAdapter, RosControlAdapter
from python_cle.tf_framework import config, TransferFunctionManager
from python_cle.cle.ClosedLoopEngine import ClosedLoopEngine

__author__ = 'GeorgHinkel'

class IntegrationTestMilestone2(unittest.TestCase):
    """
    A test case class that contains the unit test to automatically run the integration test
    """
    def run_integration_test(self):
        """
        Runs the integration test
        """
        # 1. Setup Gazebo
        # 2. Load robot model & environment model
        # 3. Stack adapters together
        ros_control = RosControlAdapter.RosControlAdapter()
        ros_comm = RosCommunicationAdapter.RosCommunicationAdapter()

        pynn_control = PyNNControlAdapter.PyNNControlAdapter()
        pynn_comm = PyNNCommunicationAdapter.PyNNCommunicationAdapter()

        tf_man = config.active_node
        assert isinstance(tf_man, TransferFunctionManager.TransferFunctionManager)

        tf_man.brain_adapter = pynn_comm
        tf_man.robot_adapter = ros_comm
        engine = ClosedLoopEngine(ros_control, pynn_control, tf_man, 0.1)

        engine.initialize()

        engine.run()

        time.sleep(10)

        engine.stop()

        engine.shutdown()

if __name__ == "main":
    unittest.main()