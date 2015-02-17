from hbp_nrp_cle.tf_framework import _Facade as nrp
from hbp_nrp_cle.tf_framework import config
from hbp_nrp_cle.tests.tf_framework.TestDevice import TestDevice
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim._MockRobotCommunicationAdapter import MockRobotCommunicationAdapter, \
    MockPublishedTopic
from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import MockBrainCommunicationAdapter

import unittest

__author__ = 'GeorgHinkel'


class Test1(unittest.TestCase):
    def test_all_right(self):

        nrp.start_new_tf_manager()

        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()

        # The annotation Neuron2Robot registers this function as a transfer function
        # As the parameter neuron0 is not explicitly mapped, the framework will assume a mapping
        # to the neuron with the GID 0 and associate with a leaky_integrator_alpha for this neuron
        @nrp.MapSpikeSink("neuron0", nrp.brain.actors[slice(0, 2, 1)], nrp.leaky_integrator_alpha,
                                v_rest=1.0, updates=[(1.0, 0.3)])
        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuron0):
            return neuron0.voltage * 1.345

        # Here is a another transfer function from neurons to robot messages
        # This time, the neuron parameter is explicitly mapped to an array of neurons
        # More precisely, the parameter is mapped to a group of __devices that are each connected to a single neuron
        # The neuron2 parameter will thus be a list of recorders
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

        brain.__dict__["actors"] = [[10, 13], [10, 14]]
        brain.__dict__["actors"] += [[42, 23, 41], [0, 8, 15]]
        brain.__dict__["actors"] += [[42, 23], [0, 8, 15]]
        brain.__dict__["sensors"] = range(45, 645)
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
