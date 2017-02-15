import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.tf_framework import TFLoadingException, TFException
from hbp_nrp_cle.tf_framework import config
from hbp_nrp_cle.tests.tf_framework.TestDevice import TestDevice
from hbp_nrp_cle.tests.tf_framework.husky import Husky
from hbp_nrp_cle.tf_framework._TransferFunction import __name__ as TFModuleName

from hbp_nrp_cle.mocks.robotsim._MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, \
    MockPublishedTopic
from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import MockBrainCommunicationAdapter
from hbp_nrp_cle.tests.tf_framework.MockBrain import MockPopulation

import unittest
from mock import MagicMock, Mock, patch
import logging
from testfixtures import log_capture, replace

__author__ = 'GeorgHinkel'


class TestTransferFunction(unittest.TestCase):

    def test_brain_source(self):
        nrp.start_new_tf_manager()
        config.brain_source = "some source"
        self.assertEqual(config.brain_source, nrp.get_brain_source())

    def test_tf_source(self):
        nrp.start_new_tf_manager()
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        @nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                                v_rest=1.0, updates=[(1.0, 0.3)])
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0):
            return neuron0.voltage * 1.345

        @nrp.Robot2Neuron()
        def transform_camera(t, camera, camera_device):
            if camera.changed:
                camera_device.inner.amplitude = 42.0

        expected_source_n2r = """@nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                        v_rest=1.0, updates=[(1.0, 0.3)])
@nrp.Neuron2Robot(Husky.RightArm.pose)
def right_arm(t, neuron0):
    return neuron0.voltage * 1.345
"""
        expected_source_r2n = """@nrp.Robot2Neuron()
def transform_camera(t, camera, camera_device):
    if camera.changed:
        camera_device.inner.amplitude = 42.0
"""

        loaded_source_n2r = config.active_node.n2r[0].source
        self.assertEqual(loaded_source_n2r, expected_source_n2r)

        loaded_source_r2n = config.active_node.r2n[0].source
        self.assertEqual(loaded_source_r2n, expected_source_r2n)

        loaded_source_n2r_and_r2n = [tf.source for tf in nrp.get_transfer_functions()]
        self.assertIn(expected_source_n2r, loaded_source_n2r_and_r2n)
        self.assertIn(expected_source_r2n, loaded_source_n2r_and_r2n)
        self.assertEqual(2, len(loaded_source_n2r_and_r2n))

    def test_tf_set(self):
        nrp.start_new_tf_manager()
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        config.active_node.brain_adapter = brain
        config.active_node.robot_adapter = robot
        config.active_node.initialize_tf = MagicMock(return_value=None)

        @nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                                v_rest=1.0, updates=[(1.0, 0.3)]
        )
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0):
            return neuron0.voltage * 1.345

        @nrp.Robot2Neuron()
        def transform_camera(t, camera, camera_device):
            if camera.changed:
                camera_device.inner.amplitude = 42.0

        tf_n2r = """@nrp.MapSpikeSink("neuron1", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                        v_rest=1.0, updates=[(3.0, 0.1)])
@nrp.Neuron2Robot()
def right_arm(t, neuron1):
    return neuron1.voltage * 2.345
"""
        tf_r2n = """@nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
@nrp.Robot2Neuron()
def transform_camera(t, camera):
    if camera.changed:
        pass
"""
        nrp.delete_transfer_function('right_arm')
        nrp.delete_transfer_function('transform_camera')
        nrp.set_transfer_function(tf_n2r, tf_n2r, 'right_arm')
        nrp.set_transfer_function(tf_r2n, tf_r2n, 'transform_camera')
        self.assertEqual(config.active_node.initialize_tf.call_count, 2)

        loaded_source_n2r_and_r2n = [tf.source for tf in nrp.get_transfer_functions()]
        self.assertIn(tf_n2r, loaded_source_n2r_and_r2n)
        self.assertIn(tf_r2n, loaded_source_n2r_and_r2n)
        number_of_tf = len(loaded_source_n2r_and_r2n)
        self.assertEqual(2, number_of_tf)

        source_update_flags = [tf.updated for tf in nrp.get_transfer_functions()]
        self.assertEqual([True] * number_of_tf, source_update_flags)

        fancy_tf = "def it_wont_work():\n return None"
        self.assertRaises(TFLoadingException, nrp.set_transfer_function, fancy_tf, fancy_tf, "it_wont_work")
        try:
            nrp.set_transfer_function(fancy_tf, fancy_tf, 'it_wont_work')
        except TFLoadingException as e:
            self.assertIn('no decorator', e.message)
            self.assertEqual('it_wont_work', e.tf_name)

        funny_tf = """@nrp.Robot2Neuron()
def transform_camera(t, camera, camera_device):
    if camera.changed:
        return Nothing!
"""
        self.assertRaises(TFLoadingException, nrp.set_transfer_function, funny_tf, funny_tf, "it_cant_work")
        try:
            nrp.set_transfer_function(funny_tf, funny_tf, 'it_cant_work')
        except TFLoadingException as e:
            self.assertIn('invalid syntax', e.message)
            self.assertIn('line 4', e.message)
            self.assertEqual('it_cant_work', e.tf_name)


    @patch('hbp_nrp_cle.tf_framework.CSVRecorder.cleanup')
    @patch('hbp_nrp_cle.tf_framework.CSVRecorder.dump_to_file')
    def test_tf_dump(self, mock_dump_to_file, mock_cleanup):

        mock_dump_to_file.return_value = ('filename', 'temp_filepath')

        nrp.start_new_tf_manager()

        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

        @nrp.MapCSVRecorder("recorder", filename = "all_joints_positions.csv", headers=["Name", "time", "Position"])
        @nrp.Robot2Neuron()
        def joint_state_monitor(t, recorder):
            recorder.record_entry("foo", t, "bar")

        nrp.initialize("MyTransferFunctions")
        self.assertEqual(mock_cleanup.call_count, 0)
        self.assertEqual([['filename','temp_filepath']], nrp.dump_csv_recorder_to_files())
        self.assertEqual(mock_cleanup.call_count, 0)
        nrp.clean_csv_recorders_files()
        self.assertEqual(mock_cleanup.call_count, 1)

    @patch('hbp_nrp_cle.tf_framework.CSVRecorder.cleanup')
    def test_tf_delete(self, mock_cleanup):

        nrp.start_new_tf_manager()

        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        config.active_node.brain_adapter = brain

        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

        @nrp.MapCSVRecorder("recorder1", filename = "1", headers=["Name", "time", "Position"])
        @nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                                v_rest=1.0, updates=[(1.0, 0.3)])
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0, recorder1):
            return neuron0.voltage * 1.345


        @nrp.MapCSVRecorder("recorder2", filename = "2", headers=["Name", "time", "Position"])
        @nrp.MapSpikeSink("neuron1", nrp.brain.actors[slice(2, 4, 1)], nrp.leaky_integrator_alpha,
                                updates=[(1.0, 0.4)], v_rest=1.0)
        @nrp.MapSpikeSink("neuron2", nrp.brain.actors[slice(4, 6, 1)], nrp.leaky_integrator_alpha,
                                updates=[(1.0, 0.4)], v_rest=1.0)
        @nrp.Neuron2Robot(Husky.LeftArm.twist)
        def left_arm_tw(t, neuron1, neuron2, recorder2):
            if neuron1.voltage < 0.678:
                if neuron2.voltage > 0.345:
                    return 0.756
                else:
                    return 1.123
            else:
                if neuron2.voltage < 0.789:
                    return 0.632
                else:
                    return 0.256

        nrp.initialize("MyTransferFunctions")

        # Delete an existing transfer function
        self.assertEqual(2, len(nrp.get_transfer_functions()))
        self.assertEqual(True, nrp.delete_transfer_function("left_arm_tw"))
        self.assertEqual(1, len(nrp.get_transfer_functions()))
        self.assertEqual(mock_cleanup.call_count, 1)
        # Try to delete it again
        self.assertEqual(False, nrp.delete_transfer_function("left_arm_tw"))
        self.assertEqual(1, len(nrp.get_transfer_functions()))
        # Delete another existing transfer function
        self.assertEqual(True, nrp.delete_transfer_function("right_arm"))
        self.assertEqual(0, len(nrp.get_transfer_functions()))
        self.assertEqual(0, len(config.active_node.brain_adapter.detector_devices))
        self.assertEqual(mock_cleanup.call_count, 2)

    @log_capture(level=logging.ERROR)
    def test_tf_publish_error(self, logcapture):

        nrp.start_new_tf_manager()

        brain = MockBrainCommunicationAdapter()
        config.active_node.brain_adapter = brain
        config.active_node.publish_error_callback = MagicMock(return_value=None)

        @nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                                v_rest=1.0, updates=[(1.0, 0.3)])
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0):
            raise Exception('foo')

        tf = nrp.get_transfer_function('right_arm')
        self.assertRaises(TFException, tf.run, 0.1)

    def test_all_right(self):

        nrp.start_new_tf_manager()

        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        @nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                                v_rest=1.0, updates=[(1.0, 0.3)])
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0):
            return neuron0.voltage * 1.345

        @nrp.MapSpikeSink("neuron1", nrp.brain.actors[slice(2, 4, 1)], nrp.leaky_integrator_alpha,
                                updates=[(1.0, 0.4)], v_rest=1.0)
        @nrp.MapSpikeSink("neuron2", nrp.brain.actors[slice(4, 6, 1)], nrp.leaky_integrator_alpha,
                                updates=[(1.0, 0.4)], v_rest=1.0)
        @nrp.Neuron2Robot(Husky.LeftArm.twist)
        def left_arm_tw(t, neuron1, neuron2):
            if neuron1.voltage < 0.678:
                if neuron2.voltage > 0.345:
                    return 0.756
                else:
                    return 1.123
            else:
                if neuron2.voltage < 0.789:
                    return 0.632
                else:
                    return 0.256

        # Here is an example of a transfer function mapping robot sensor data to spikes
        # As the image processing is a common task, this is done through a specialized
        # device in the neuronal simulator. However, this device might not be mapped to
        # physical Nest device, but do some processing internally and use a less specialized
        # device type internally
        @nrp.MapRobotSubscriber("camera", Husky.Eye.camera)
        @nrp.MapSpikeSource("camera_device", nrp.brain.sensors[slice(0, 600, 1)],
                                TestDevice())
        @nrp.Robot2Neuron()
        def transform_camera(t, camera, camera_device):
            if camera.changed:
                camera_device.inner.amplitude = 42.0

        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

        brain.__dict__["actors"] = MockPopulation(range(0, 60))
        brain.__dict__["sensors"] = MockPopulation(range(45, 645))
        config.brain_root = brain

        nrp.initialize("MyTransferFunctions")

        husky_right_arm = right_arm.topic
        husky_left_arm = left_arm_tw.topic

        camera = transform_camera.camera
        camera_device = transform_camera.camera_device

        brain.refresh_buffers(0.5)
        robot.refresh_buffers(0.5)
        config.active_node.run_neuron_to_robot(0.5)
        config.active_node.run_robot_to_neuron(0.5)

        camera.value = "Definitely not an image"

        brain.refresh_buffers(1.5)
        robot.refresh_buffers(1.5)
        config.active_node.run_neuron_to_robot(1.5)
        config.active_node.run_robot_to_neuron(1.5)

        assert isinstance(husky_right_arm, MockPublishedTopic)
        assert isinstance(husky_left_arm, MockPublishedTopic)

        assert len(husky_right_arm.sent) == 2
        assert len(husky_left_arm.sent) == 2

        assert husky_right_arm.sent[0] == 1.345
        assert husky_right_arm.sent[1] == 1.345 * 0.3

        assert husky_left_arm.sent[0] == 0.256
        assert husky_left_arm.sent[1] == 0.756

        assert camera_device.inner.amplitude == 42.0

        config.active_node.reset()

if __name__ == "__main__":
    unittest.main()
