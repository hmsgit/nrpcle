"""
Contains a library of transfer functions
"""

# import sensor_msgs.msg
from __future__ import division
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import logging


__author__ = 'GeorgHinkel, AlessandroAmbrosano'

bridge = CvBridge()
logger = logging.getLogger(__name__)


class Camera(object):
    """
    Utility class for converting between measures in the Field of View
    """

    def __init__(self):
        # sx and sy denote the image size (px)
        # note: not to be confused with the number of pixels per millimeter on the camera sensor.
        self._calib_sx = 320
        self._calib_sy = 240
        # fx and fy denote the focal length (px)
        # (physical focal length (mm) * pixel per mm on the sensor in axis x/y (px/mm))
        # if the pixels are square (which we assume being the case) the two values are equivalent.
        self._calib_fx = 160
        self._calib_fy = 160
        # cx and cy denote the pixel coordinates of the center pixel on the camera (px)
        self._calib_cx = 160
        self._calib_cy = 120

        # cx, cy, fx and fy are published on the camera_info topic of the camera at the "K" entry.
        # K denotes the camera matrix and is structured as follows
        # | fx  0.  cx |
        # | 0.  fy  cy |
        # | 0.  0.  1. |

        self._curr_cx = self._curr_cy = None
        self._curr_fx = self._curr_fy = None
        self._curr_sx = self._curr_sy = None

        self.set_image_size(320, 240)

    def set_image_size(self, w, h):
        """
        Sets the image size

        :param w: image's new width
        :param h: image's new height
        """
        if w <= 0 or h <= 0:
            return False
        self._curr_sx = w
        self._curr_sy = h
        scale_x = float(self._curr_sx) / self._calib_sx
        scale_y = float(self._curr_sy) / self._calib_sy
        self._curr_fx = self._calib_fx * scale_x
        self._curr_cx = self._calib_cx * scale_x
        self._curr_fy = self._calib_fy * scale_y
        self._curr_cy = self._calib_cy * scale_y
        return True

    def pixel2metric(self, u, v):
        """
        Converts a pixel coordinate on the image to a metric distance from its center.

        :param u: the x coordinate (column) of the pixel (px)
        :param v: the y coordinate (row) of the pixel (px)
        :return: a pair (xm, ym) denoting the metric distance from the center of the image
            (pure number)
        """
        return float(u - self._curr_cx) / self._curr_fx, float(v - self._curr_cy) / self._curr_fy

    def metric2pixel(self, xm, ym):
        """
        Converts a metric distance from the image center to a pixel coordinate.

        :param xm: the x distance from the center (pure number)
        :param ym: the y distance from the center (pure number)
        :return: a pair (u, v) denoting a pixel's coordinates (px)
        """
        return xm * self._curr_fx + self._curr_cx, ym * self._curr_fy + self._curr_cy

    @staticmethod
    def metric2angle(xm, ym):
        """
        Converts a metric distance from the image's center to an (azimuth, elevation) pair.

        :param xm: the x distance from the center (pure number)
        :param ym: the y distance from the center (pure number)
        :return: a pair (a, e) denoting the azimuth and the elevation, the center being (0, 0)
            (deg)
        """
        _rho = math.sqrt(xm ** 2 + ym ** 2 + 1)
        a = -math.atan(xm) * 180 / np.pi  # azimuth
        e = -math.asin(ym / _rho) * 180 / np.pi  # elevation
        return a, e

    @staticmethod
    def angle2metric(a, e):
        """
        Converts an (azimuth, elevation) pair to a metric distance from the image's center.

        :param a: the azimuth angle from the image's center (deg)
        :param e: the elevation angle from the image's center (deg)
        :return: a pair (xm, ym) denoting the metric distance from the center of the image
            (pure number)
        """
        xm = -math.tan(a * np.pi / 180)
        ym = -math.tan(e * np.pi / 180) * math.sqrt(xm ** 2 + 1)
        return xm, ym

    def pixel2angle(self, u, v):
        """
        Converts a pixel coordinate on the image to an (azimuth, elevation) pair.

        :param u: the x coordinate (column) of the pixel (px)
        :param v: the y coordinate (row) of the pixel (px)
        :return: a pair (a, e) denoting the azimuth and the elevation, the center being (0, 0)
            (deg)
        """
        xm, ym = self.pixel2metric(u, v)
        a, e = Camera.metric2angle(xm, ym)
        return a, e

    def angle2pixel(self, a, e):
        """
        Converts an (azimuth, elevation) pair to a pixel coordinate on the image.

        :param a: the azimuth angle from the image's center (deg)
        :param e: the elevation angle from the image's center (deg)
        :return: a pair (u, v) denoting a pixel's coordinates (px)
        """
        xm, ym = Camera.angle2metric(a, e)
        u, v = self.metric2pixel(xm, ym)
        return u, v

    def pixel2norm(self, u, v):
        """
        Converts a pixel coordinate on the image to a normalized distance from its center.

        :param u: the x coordinate (column) of the pixel (px)
        :param v: the y coordinate (row) of the pixel (px)
        :return: a pair (x, y) denoting the distance from the center of the image
            (pure number in range [-1, 1] x [-1, 1])
        """
        x = 2 * u / self._curr_sx - 1
        y = 2 * v / self._curr_sy - 1
        return x, y

    def norm2pixel(self, x, y):
        """
        Converts a normalized distance from the image's center to the corresponding pixel.

        :param x: the x coordinate of the distance from the center (pure number in range [-1, 1])
        :param y: the y coordinate of the distance from the center (pure number in range [-1, 1])
        :return: a pair (u, v) denoting a pixel's coordinates (px)
        """
        u = self._curr_sx * (x + 1) / 2
        v = self._curr_sy * (y + 1) / 2
        return u, v

    def norm2angle(self, x, y):
        """
        Converts a normalized distance from the image's center to the corresponding
            (azimuth, elevation) pair.

        :param x: the x coordinate of the distance from the center (pure number in range [-1, 1])
        :param y: the y coordinate of the distance from the center (pure number in range [-1, 1])
        :return: a pair (a, e) denoting the azimuth and the elevation, the center being (0, 0)
            (deg)
        """
        u, v = self.norm2pixel(x, y)
        a, e = self.pixel2angle(u, v)
        return a, e

    def norm2metric(self, x, y):
        """
        Converts a normalized distance from the image's center to a metric distance from its center

        :param x: the x coordinate of the distance from the center (pure number in range [-1, 1])
        :param y: the y coordinate of the distance from the center (pure number in range [-1, 1])
        :return: a pair (xm, ym) denoting the metric distance from the center of the image
            (pure number)
        """
        u, v = self.norm2pixel(x, y)
        xm, ym = self.pixel2metric(u, v)
        return xm, ym

    @property
    def height(self):
        """
        Gets the image's height (px)
        """
        return self._curr_sy

    @property
    def width(self):
        """
        Gets the image's width (px)
        """
        return self._curr_sx

    @property
    def cal_height(self):
        """
        Gets the image's height computed during calibration (px)
        """
        return self._calib_sy

    @property
    def cal_width(self):
        """
        Gets the image's width computed during calibration (px)
        """
        return self._calib_sx

    @property
    def fx(self):
        """
        Gets the image's x focal length (px)
        """
        return self._curr_fx

    @property
    def fy(self):
        """
        Gets the image's y focal length (px)
        """
        return self._curr_fy

    @property
    def cx(self):
        """
        Gets the image center's x coordinate (px)
        """
        return self._curr_cx

    @property
    def cy(self):
        """
        Gets the image center's y coordinate (px)
        """
        return self._curr_cy

    @property
    def cal_fx(self):
        """
        Gets the image's x focal length computed during calibration (px)
        """
        return self._calib_fx

    @property
    def cal_fy(self):
        """
        Gets the image's y focal length computed during calibration (px)
        """
        return self._calib_fy

    @property
    def cal_cx(self):
        """
        Gets the image center's x coordinate computed during calibration (px)
        """
        return self._calib_cx

    @property
    def cal_cy(self):
        """
        Gets the image center's y coordinate computed during calibration (px)
        """
        return self._calib_cy


cam = Camera()


def find_centroid_hsv(image, lower, upper):
    """
    Finds the centroid of the pixels in an image lying in a given HSV slice.

    :param image: A Gazebo ROS image (sensor_msgs.msg.Image)
    :param lower: The lower value of the HSV slice we want to detect (a 3 elements int list with
        values in range 0-255, refer to cv2 documentation for more details).
    :param upper: The upper value of the HSV slice we want to detect (a 3 elements int list with
        values in range 0-255, refer to cv2 documentation for more details).
    :returns: a pair (x, y) denoting the image pixel where the centroid lies, or None if anything
        goes wrong.
    """

    try:
        img_in = bridge.imgmsg_to_cv2(image)
        hsv_im = cv2.cvtColor(img_in, cv2.COLOR_RGB2HSV)
        lower_np = np.array(lower, dtype="uint8")
        upper_np = np.array(upper, dtype="uint8")
        mask = cv2.inRange(hsv_im, lower_np, upper_np)
        img_out = cv2.bitwise_and(img_in, img_in, mask=mask)

        a = cv2.findNonZero(cv2.cvtColor(img_out, cv2.COLOR_RGB2GRAY))

        # At least one point in the HSV slice is detected in the image
        if a is not None and a.size != 0:
            b = np.array(
                    [[float(x[0][0]) / (a.size / 2), float(x[0][1]) / (a.size / 2)] for x in a]
                ).transpose()
            y_c, x_c = int(cv2.sumElems(b[1])[0]), int(cv2.sumElems(b[0])[0])
            return x_c, y_c

        return None
    # pylint: disable=broad-except
    except Exception as e:
        logger.error("%s", str(e))
        return None


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
