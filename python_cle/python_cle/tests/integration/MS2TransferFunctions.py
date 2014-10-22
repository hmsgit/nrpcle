"""
The transfer functions required by Milestone 2
"""

__author__ = 'GeorgHinkel'

from python_cle.tests.integration import Husky
from geometry_msgs.msg import Twist, Vector3
import python_cle.tf_framework as nrp
import itertools

MAX_SPIKE_RATE = 8.  # Hz
WHEEL_SCALING_FACTOR = .123


@nrp.MapNeuronParameter("left_wheel_neuron", nrp.brain.actors[1],
                        nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot(Husky.left_wheel_twist)
def left_wheels_twist(t, left_wheel_neuron):
    """
    Transfers the voltage from the first action neuron to a twist message for the robot
    :param left_wheel_neuron: A voltmeter device instrumenting action neuron 0
    :param t: The simulation time
    :return: A twist message to be sent to the left wheel
    """
    return Twist(angular=Vector3(WHEEL_SCALING_FACTOR * left_wheel_neuron.voltage, 0, 0))


@nrp.MapNeuronParameter("right_wheel_neuron", nrp.brain.actors[2],
                        nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot(Husky.right_wheel_twist)
def right_wheels_twist(t, right_wheel_neuron):
    """
    Transfers the voltage from the second action neuron to a twist message for the robot
    :param right_wheel_neuron: A voltmeter device instrumenting action neuron 2
    :param t: The simulation time
    :return: A twist message to be sent to the right wheel
    """
    return Twist(linear=Vector3(WHEEL_SCALING_FACTOR * right_wheel_neuron.voltage, 0, 0))

LEFT_PATTERN = [nrp.brain.sensors[2], nrp.brain.sensors[0], nrp.brain.sensors[0]]
RIGHT_PATTERN = [nrp.brain.sensors[2], nrp.brain.sensors[1], nrp.brain.sensors[1]]

LEFT_IMAGE = [sensor for pattern in itertools.repeat(LEFT_PATTERN, 20) for sensor in pattern]
RIGHT_IMAGE = [sensor for pattern in itertools.repeat(RIGHT_PATTERN, 20) for sensor in pattern]

ROW_SENSORS = list(itertools.chain(LEFT_IMAGE, RIGHT_IMAGE))
ALL_SENSORS = [sensor for row in itertools.repeat(ROW_SENSORS, 20) for sensor in row]

@nrp.MapRobotParameter("eye_sensor", Husky.eye_sensor)
@nrp.MapNeuronParameter("eye_devices", ALL_SENSORS, nrp.poisson)
@nrp.Robot2Neuron()
def left_eye_sensor_transmit(t, eye_sensor, eye_devices):
    """
    Transfers the image data from the eye sensor to the neuronal simulator
    :param eye_sensor: The sensor topic from the left eye
    :param t: The simulation time
    :param eye_devices: The Poisson generators for all the sensors
    """
    if eye_sensor.changed:
        eye_devices.rate = eye_sensor.value.data / 256.0 * MAX_SPIKE_RATE