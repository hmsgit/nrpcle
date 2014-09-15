from tf_framework import tf_framework as nrp
from tf_framework import config
from tests.husky import Husky

from mocks.robotsim.MockRobotCommunicationAdapter import MockRobotCommunicationAdapter
#from mocks.brainsim.MockBrainCommunicationAdapter import MockBrainCommunicationAdapter
from brainsim.PyNNCommunicationAdapter import PyNNCommunicationAdapter
import pyNN.nest as sim

__author__ = 'DimitriProbst'

# Transfer functions may in general have some state
# in this case, we reserve memory for a global double variable
right_arm_v = 0

# The annotation Neuron2Robot registers this function as a transfer function
# As the parameter neuron0 is not explicitly mapped, the framework will assume a mapping
# to the neuron with the GID 0 and associate with a voltmeter for this neuron
@nrp.Neuron2Robot(Husky.RightArm.pose)
def right_arm(t, neuron0):
    return neuron0.voltage * 1.345


## Here is a another transfer function from neurons to robot messages
## This time, the neuron parameter is explicitly mapped to an array of neurons
## More precisely, the parameter is mapped to a group of devices that are each connected to a single neuron
## The neuron2 parameter will thus be a list of recorders
#population = sim.Population(10, sim.IF_curr_alpha)
#@nrp.MapNeuronParameter("neuron2", population, nrp.voltmeter)
#@nrp.Neuron2Robot(Husky.LeftArm.twist)
#def left_arm_tw(t, neuron1, neuron2):
#    if neuron1.voltage < 0.678:
#        if neuron2[0].voltage > 0.345:
#            return 0.756
#        else:
#            return 1.123
#    else:
#        if neuron2[1].voltage < 0.789:
#            return 0.755
#        else:
#            return 0.256
#
#
## Here is an example of a transfer function mapping robot sensor data to spikes
## As the image processing is a common task, this is done through a specialized
## device in the neuronal simulator. However, this device might not be mapped to
## physical Nest device, but do some processing internally and use a less specialized
## device type internally
#@nrp.MapRobotParameter("camera", Husky.Eye.camera)
#@nrp.MapNeuronParameter("camera_device", range(45, 645), MonochromeImageSpikeGenerator(20, 30))
#@nrp.Robot2Neuron()
#def transform_camera(t, camera, camera_device):
#    print("transform_camera called!")
#    if camera.changed:
#        camera_device.update_image(camera.value)


if __name__ == "__main__":
    sim.setup(timestep=0.1, min_delay=0.1, max_delay=4.0,
              threads=1, rng_seeds=[23])

    nrp.set_nest_adapter(PyNNCommunicationAdapter())
    nrp.set_robot_adapter(MockRobotCommunicationAdapter())
    nrp.initialize("MyTransferFunctions")
    sim.run(0.5)
#    config.active_node.run_neuron_to_robot(0.5)
    config.active_node.run_robot_to_neuron(0.5)
    sim.end()
