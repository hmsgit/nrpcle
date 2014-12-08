"""
Execute integration test as a unit test
"""

import unittest
import time
from hbp_nrp_cle.tests.integration import PyNNScript
from hbp_nrp_cle.brainsim import PyNNCommunicationAdapter, PyNNControlAdapter
from hbp_nrp_cle.robotsim import RosCommunicationAdapter, RosControlAdapter
from hbp_nrp_cle.tf_framework import config, _TransferFunctionManager
from hbp_nrp_cle.cle.ClosedLoopEngine import ClosedLoopEngine
# pylint: disable=W0401
from hbp_nrp_cle.tests.integration import MS2TransferFunctions

import os
import platform

__author__ = 'GeorgHinkel'

ROS_MASTER_URI = "ROS_MASTER_URI"

class IntegrationTestMilestone2(unittest.TestCase):
    """
    A test case class that contains the unit test to automatically run the integration test
    """
    def test_integration(self):
        """
        Runs the integration test
        """
        # 1. Setup Gazebo
        # TODO: Make sure Gazebo is started
        env = os.environ
        ros_master_uri = env[ROS_MASTER_URI] if ROS_MASTER_URI in env else None
        if ros_master_uri is None:
            env[ROS_MASTER_URI] = "http://" + platform.node() + ":11311/"
        # 2. Load robot model & environment model
        # TODO: Load robot model and environment model
        # 3. Stack adapters together
        ros_control = RosControlAdapter.RosControlAdapter()
        ros_comm = RosCommunicationAdapter.RosCommunicationAdapter()

        pynn_control = PyNNControlAdapter.PyNNControlAdapter()
        pynn_comm = PyNNCommunicationAdapter.PyNNCommunicationAdapter()

        pynn_control.initialize()

        PyNNScript.init_brain_simulation()

        tf_man = config.active_node
        assert isinstance(tf_man, _TransferFunctionManager.TransferFunctionManager)

        tf_man.brain_adapter = pynn_comm
        tf_man.robot_adapter = ros_comm
        engine = ClosedLoopEngine(ros_control, pynn_control, tf_man, 0.1)

        engine.initialize()

        engine.start()

        time.sleep(10)

        engine.stop()

        engine.shutdown()

if __name__ == "__main__":
    unittest.main()