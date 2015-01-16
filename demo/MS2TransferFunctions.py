"""
The transfer functions required by Milestone 2
"""

__author__ = 'GeorgHinkel'

import Husky
from geometry_msgs.msg import Twist, Vector3
import hbp_nrp_cle.tf_framework as nrp
from cv_bridge import CvBridge

MAX_SPIKE_RATE = 1000.  # Hz
WHEEL_SCALING_FACTOR = 100.
bridge = CvBridge()


@nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[0],
                   nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[1],
                   nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot(Husky.wheel_twist)
# pylint: disable=W0613
def linear_twist(t, left_wheel_neuron, right_wheel_neuron):
    """
    Transfers the voltage of the leaky integrators to a linear and angular twist message
    :param left_wheel_neuron: Action neuron 1
    :param right_wheel_neuron: Action neuron 2
    :param t: The simulation time
    :return: A twist message to be sent to the left wheel
    """
    ROBOT_WIDTH = 0.67  # m
    lw = left_wheel_neuron.voltage
    rw = right_wheel_neuron.voltage
    linear = Vector3(WHEEL_SCALING_FACTOR * min((lw, rw)), 0, 0)
    angular = Vector3(0, 0, WHEEL_SCALING_FACTOR * (rw - lw) * 2.0 / ROBOT_WIDTH)
    return Twist(linear=linear, angular=angular)

# LEFT_PATTERN = [nrp.brain.sensors[2], nrp.brain.sensors[0], nrp.brain.sensors[0]]
# RIGHT_PATTERN = [nrp.brain.sensors[2], nrp.brain.sensors[1], nrp.brain.sensors[1]]
#
# LEFT_IMAGE = [sensor for pattern in itertools.repeat(LEFT_PATTERN, 20) for sensor in pattern]
# RIGHT_IMAGE = [sensor for pattern in itertools.repeat(RIGHT_PATTERN, 20) for sensor in pattern]
#
# ROW_SENSORS = list(itertools.chain(LEFT_IMAGE, RIGHT_IMAGE))
# ALL_SENSORS = [sensor for row in itertools.repeat(ROW_SENSORS, 20) for sensor in row]


@nrp.MapRobotSubscriber("eye_sensor", Husky.eye_sensor)
@nrp.MapSpikeSource("red_left_eye_device", nrp.brain.sensors[0], nrp.poisson)
@nrp.MapSpikeSource("red_right_eye_device", nrp.brain.sensors[1], nrp.poisson)
@nrp.MapSpikeSource("green_blue_eye_device", nrp.brain.sensors[2], nrp.poisson)
@nrp.Robot2Neuron()
# pylint: disable=W0613
def eye_sensor_transmit(t, eye_sensor, red_left_eye_device, red_right_eye_device,
                        green_blue_eye_device):
    """
    Transfers the image data from the eye sensor to the neuronal simulator
    :param eye_sensor: The sensor topic from the left eye
    :param t: The simulation time
    :param red_left_eye_device: The Poisson generator for the left red eye
    :param red_right_eye_device: The Poisson generator for the right red eye
    :param green_blue_eye_device: The Poisson generator for the green-blue eye device
    """
    red_left_rate = 0.
    red_right_rate = 0.
    green_blue_rate = 0.
    if not isinstance(eye_sensor.value, type(None)):    # Problem: starts as NoneType
#        print eye_sensor.changed
        cv_image = bridge.imgmsg_to_cv2(eye_sensor.value, "rgb8") / 256.
        cv_image = 5000 ** cv_image / 5000
        for i in range(0, cv_image.shape[0]):
            for j in range(0, cv_image.shape[1]):
#                # TODO: Implement how devices are configured based on the image data
                if j < cv_image.shape[1] / 2:
                    red_left_rate += cv_image[i, j, 0]
                else:
                    red_right_rate += cv_image[i, j, 0]
                green_blue_rate += cv_image[i, j, 1]
                green_blue_rate += cv_image[i, j, 2]
        red_left_rate *= (MAX_SPIKE_RATE / cv_image.size)
        red_right_rate *= (MAX_SPIKE_RATE / cv_image.size)
        green_blue_rate *= (MAX_SPIKE_RATE / cv_image.size)
#    print np.array([red_left_rate, red_right_rate, green_blue_rate])
#    red_left_rate = (8. / 256.) * SCALING_FACTOR
#    red_right_rate = 0.  # (8. / 256.) * SCALING_FACTOR
#    green_blue_rate = 0.  # (8. / 256.) * SCALING_FACTOR

    red_left_eye_device.rate = red_left_rate
    red_right_eye_device.rate = red_right_rate
    green_blue_eye_device.rate = green_blue_rate
