"""
Contains a library of transfer functions
"""

# import sensor_msgs.msg
from cv_bridge import CvBridge

__author__ = 'GeorgHinkel'

bridge = CvBridge()


def detect_red(image):
    """
    Performs a very simple image detection as used in the Braitenberg demo.
    An incoming image is analyzed per pixel. If r > (g + b), i.e. if the color is quite red as
    according to the HSV color model, then pixel is regarded as red.

    :param image: The image
    :returns: An object with three properties:
        - *left*: This is the percentage of red pixels in the left half of the image
        - *right*: This is the percentage of red pixels in the right half of the image
        - *go_on*: This is the percentage of non-red pixels of the overall image

    :example: A completely red image (255,0,0) results in (1,1,0)
    :example: A completely yellow image (255,255,0) results in (0,0,1)

    The lightest color that is recognized as red is (255,127,127).
    """
    # assert isinstance(image, sensor_msgs.msg.Image)
    red_left_rate = 0.
    red_right_rate = 0.
    green_blue_rate = 0.
    if not isinstance(image, type(None)):  # Problem: starts as NoneType
        # print eye_sensor.changed
        # load image in [0,1]
        cv_image = bridge.imgmsg_to_cv2(image, "rgb8") / 256.
        # intensify values but keep in [0,1]
        cv_image = 5000 ** cv_image / 5000

        for i in range(0, cv_image.shape[0]):
            for j in range(0, cv_image.shape[1]):
                r = cv_image[i, j, 0]
                g = cv_image[i, j, 1]
                b = cv_image[i, j, 2]

                if r > (g + b):
                    if j < cv_image.shape[1] / 2:
                        red_left_rate += 1.
                    else:
                        red_right_rate += 1.
                else:
                    green_blue_rate += 1.

        red_left_rate *= (6. / cv_image.size)
        red_right_rate *= (6. / cv_image.size)
        green_blue_rate *= (3. / cv_image.size)

        print "red_left_rate: ", red_left_rate
        print "red_right_rate: ", red_right_rate
        print "green_blue_rate: ", green_blue_rate

    class __results(object):
        """
        An intermediate helper class for the results of detect_red
        """
        def __init__(self, left, right, go_on):
            self.left = left
            self.right = right
            self.go_on = go_on

    return __results(red_left_rate, red_right_rate, green_blue_rate)
