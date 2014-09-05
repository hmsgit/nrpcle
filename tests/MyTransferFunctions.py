from tf_framework import tf_framework as nrp
from tf_framework.spike_generators.CameraSpikeGenerator import CameraSpikeGenerator
from tests.husky import Husky

from mocks.robotsim.MockRobotCommunicationAdapter import MockRobotCommunicationAdapter
from mocks.brainsim.MockBrainCommunicationAdapter import MockBrainCommunicationAdapter

__author__ = 'GeorgHinkel'

# Transfer functions may in general have some state
# in this case, we reserve memory for a global double variable
right_arm_v = 0


# The annotation Neuron2Robot registers this function as a transfer function
# As the parameter neuron0 is not explicitly mapped, the framework will assume a mapping
# to the neuron with the GID neuron0
@nrp.Neuron2Robot(Husky.RightArm.pose)
def right_arm(t, neuron0):
    return neuron0.voltage * 1.345


# Here is a another transfer function from neurons to robot messages
# This time, the neuron parameter is explicitly mapped to an array of neurons
@nrp.MapNeuronParameter("neuron2", nrp.spikes("neuron2", 10), nrp.recorder)
@nrp.Neuron2Robot(Husky.LeftArm.twist)
def left_arm_tw(t, neuron1, neuron2):
    if neuron1.voltage < 0.678:
        if neuron2[0]:
            return 0
        else:
            return 1
    else:
        if neuron2[1]:
            return 0.75
        else:
            return 0.25


# Here is an example of a transfer function mapping robot sensor data to spikes
# As the image processing is a common task, this is done through a specialized
# device in the neuronal simulator. However, this device might not be mapped to
# physical Nest device, but do some processing internally and use a less specialized
# device type internally
@nrp.MapRobotParameter("camera", Husky.Eye.camera)
@nrp.MapNeuronParameter("camera_device", nrp.spikes("neuron45", 200000), CameraSpikeGenerator(200, 300))
@nrp.Robot2Neuron()
def transform_camera(t, camera, camera_device):
    if camera.changed:
        camera_device.update_image(camera.value)


if __name__ == "__main__":
    nrp.set_nest_adapter(MockBrainCommunicationAdapter())
    nrp.set_robot_adapter(MockRobotCommunicationAdapter())
    nrp.initialize("MyTransferFunctions")