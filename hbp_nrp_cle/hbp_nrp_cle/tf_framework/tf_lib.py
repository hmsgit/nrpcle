"""
Contains a library of transfer functions
"""

import sensor_msgs.msg
from cv_bridge import CvBridge

__author__ = 'GeorgHinkel'

bridge = CvBridge()


def detect_red(image):
    """
    Detects a red image
    :param image: The image
    """
    assert isinstance(image, sensor_msgs.msg.Image)
    red_left_rate = 0.
    red_right_rate = 0.
    green_blue_rate = 0.
    if not isinstance(image, type(None)):  # Problem: starts as NoneType
        # print eye_sensor.changed
        cv_image = bridge.imgmsg_to_cv2(image, "rgb8")
        for i in range(0, cv_image.shape[0]):
            for j in range(0, cv_image.shape[1]):
                if j < cv_image.shape[1] / 2:
                    red_left_rate += cv_image[i, j, 0]
                else:
                    red_right_rate += cv_image[i, j, 0]
                green_blue_rate += cv_image[i, j, 1]
                green_blue_rate += cv_image[i, j, 2]

    class __results(object):
        """
        An intermediate helper class for the results of detect_red
        """
        def __init__(self, left, right, go_on):
            self.left = left
            self.right = right
            self.go_on = go_on

    return __results(red_left_rate, red_right_rate, green_blue_rate)
