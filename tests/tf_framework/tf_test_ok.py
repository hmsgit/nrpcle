from tf_framework import tf_framework as nrp
from tf_framework import config
from tf_framework.spike_generators.MonochromeImageSpikeGenerator import MonochromeImageSpikeGenerator
from tests.husky import Husky

from mocks.robotsim.MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, MockPublishedTopic
from mocks.brainsim.MockBrainCommunicationAdapter import MockBrainCommunicationAdapter

import unittest

__author__ = 'GeorgHinkel'

class Test1(unittest.TestCase):

    def test_all_right(self):

        nrp.start_new_tf_manager()

        # The annotation Neuron2Robot registers this function as a transfer function
        # As the parameter neuron0 is not explicitly mapped, the framework will assume a mapping
        # to the neuron with the GID 0 and associate with a voltmeter for this neuron
        @nrp.MapNeuronParameter("neuron0", [10], nrp.voltmeter, updates=[(1.0, 0.3)])
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0):
            return neuron0.voltage * 1.345

        # Here is a another transfer function from neurons to robot messages
        # This time, the neuron parameter is explicitly mapped to an array of neurons
        # More precisely, the parameter is mapped to a group of devices that are each connected to a single neuron
        # The neuron2 parameter will thus be a list of recorders
        @nrp.MapNeuronParameter("neuron2", [[42], [23], [0], [8], [15]], nrp.voltmeter, updates=[(1.0, 0.4)])
        @nrp.Neuron2Robot(Husky.LeftArm.twist)
        def left_arm_tw(t, neuron1, neuron2):
            if neuron1.voltage < 0.678:
                if neuron2[0].voltage > 0.345:
                    return 0.756
                else:
                    return 1.123
            else:
                if neuron2[1].voltage < 0.789:
                    return 0.755
                else:
                    return 0.256

        # Here is an example of a transfer function mapping robot sensor data to spikes
        # As the image processing is a common task, this is done through a specialized
        # device in the neuronal simulator. However, this device might not be mapped to
        # physical Nest device, but do some processing internally and use a less specialized
        # device type internally
        @nrp.MapRobotParameter("camera", Husky.Eye.camera)
        @nrp.MapNeuronParameter("camera_device", range(45, 645), MonochromeImageSpikeGenerator(20, 30))
        @nrp.Robot2Neuron()
        def transform_camera(t, camera, camera_device):
            if camera.changed:
                camera_device.update_image(camera.value)

        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)
        nrp.initialize("MyTransferFunctions")

        husky_right_arm = robot.register_publish_topic(Husky.RightArm.pose)
        husky_left_arm = robot.register_publish_topic(Husky.LeftArm.twist)

        config.active_node.run_neuron_to_robot(0.5)
        config.active_node.run_robot_to_neuron(0.5)
        config.active_node.run_neuron_to_robot(1.5)
        config.active_node.run_robot_to_neuron(1.5)

        assert isinstance(husky_right_arm, MockPublishedTopic)
        assert isinstance(husky_left_arm, MockPublishedTopic)

        assert len(husky_right_arm.sent) == 2
        assert len(husky_left_arm.sent) == 2

        assert husky_right_arm.sent[0] == 1.345
        assert husky_right_arm.sent[1] == 1.345 * 0.3

        assert husky_left_arm.sent[0] == 0.256
        assert husky_left_arm.sent[1] == 0.755