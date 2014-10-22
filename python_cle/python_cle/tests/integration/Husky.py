"""
The Husky robot model used by milestone 2
"""

from python_cle.robotsim.RobotInterface import Topic, PreprocessedTopic
import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np

__author__ = 'GeorgHinkel'

right_wheel_twist = Topic("/husky1/right_wheel/twist", geometry_msgs.msg.Twist)
left_wheel_twist = Topic("/husky1/left_wheel/twist", geometry_msgs.msg.Twist)


def transform_image_to_numpy(image):
    """
    Transforms an incoming image message to a numpy array
    :param image: The image data
    :type image: sensor_msgs.msg.Image
    :return: A numpy array with the image data
    """
    assert isinstance(image, sensor_msgs.msg.Image)
    array = np.asarray(image.data)
    array = np.array(array, dtype=np.float64)
    if array.max() != 0:
       array /= array.max()
    return array

eye_sensor = PreprocessedTopic("/husky1/eye/image", sensor_msgs.msg.Image, transform_image_to_numpy)
