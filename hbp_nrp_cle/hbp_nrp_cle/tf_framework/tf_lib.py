"""
Contains a library of transfer functions
"""

# import sensor_msgs.msg
from __future__ import division
from cv_bridge import CvBridge
import cv2
import numpy as np

__author__ = 'GeorgHinkel'

bridge = CvBridge()


def detect_red(image):
    """
    Performs a very simple image detection as used in the Braitenberg demo.

    :param image: The image
    :returns: An object with three properties:
        - *left*: This is the percentage of red pixels in the left half of the image
        - *right*: This is the percentage of red pixels in the right half of the image
        - *go_on*: This is the percentage of non-red pixels of the overall image

    :example: A completely red image (255,0,0) results in (1,1,0)
    :example: A completely yellow image (255,255,0) results in (0,0,1)

    The lightest color that is recognized as red is (255,127,127).
    """
    red_left = red_right = green_blue = 0
    if not isinstance(image, type(None)):
        lower_red = np.array([0, 30, 30])
        upper_red = np.array([0, 255, 255])
        cv_image = bridge.imgmsg_to_cv2(image, "rgb8")
        # Transform image to HSV (easier to detect colors).
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        # Create a mask where every non red pixel will be a Zero.
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        image_size = (cv_image.shape[0] * cv_image.shape[1])
        if (image_size > 0):
            half = cv_image.shape[1] // 2
            # Get the number of red pixels in the image.
            red_left = cv2.countNonZero(mask[:, :half])
            red_right = cv2.countNonZero(mask[:, half:])
            green_blue = (image_size - (red_left + red_right)) / image_size
            # We have to mutiply the rate by two since it is for an half image only.
            red_left = 2 * (red_left / image_size)
            red_right = 2 * (red_right / image_size)

    class __results(object):
        """
        An intermediate helper class for the results of detect_red
        """

        def __init__(self, left, right, go_on):
            self.left = left
            self.right = right
            self.go_on = go_on

    return __results(red_left, red_right, green_blue)


def get_color_values(image, width=40, height=30):
    """
    Gets the color values of an image for use in the Braitenberg demo.
    An incoming image is resized and then analyzed per pixel. All the red, green and blue values
    of all pixels are returned as a result

    :param image: The image
    :returns: An object with the color channels of the image separated in two image halves, each

    The lightest color that is recognized as red is (255,127,127).
    """
    # assert isinstance(image, sensor_msgs.msg.Image)
    # pixel_values = np.zeros((30, 30, 3), np.float64)
    half_width = width / 2
    n = half_width * height
    red_left_rate = np.zeros(n)
    green_left_rate = np.zeros(n)
    blue_left_rate = np.zeros(n)
    red_right_rate = np.zeros(n)
    green_right_rate = np.zeros(n)
    blue_right_rate = np.zeros(n)
    if not isinstance(image, type(None)):  # Problem: starts as NoneType
        # print eye_sensor.changed
        # load image in [0,1]
        cv_image = bridge.imgmsg_to_cv2(image, "rgb8") / 256.
        # resize, then intensify values but keep in [0,1]
        cv_image = cv2.resize(cv_image, (width, height))
        cv_image = 5000 ** cv_image / 5000

        red_left_rate = cv_image[:, 0:half_width, 0].flatten()
        green_left_rate = cv_image[:, 0:half_width, 1].flatten()
        blue_left_rate = cv_image[:, 0:half_width, 2].flatten()
        red_right_rate = cv_image[:, half_width:width, 0].flatten()
        green_right_rate = cv_image[:, half_width:width, 1].flatten()
        blue_right_rate = cv_image[:, half_width:width, 2].flatten()

    class __results(object):
        """
        An intermediate helper class for the results of detect_red
        """

        def __init__(self, left_red, left_green, left_blue, right_red, right_green, right_blue):
            self.left_red = left_red
            self.left_green = left_green
            self.left_blue = left_blue
            self.right_red = right_red
            self.right_green = right_green
            self.right_blue = right_blue

    return __results(red_left_rate, green_left_rate, blue_left_rate, red_right_rate,
                     green_right_rate, blue_right_rate)
