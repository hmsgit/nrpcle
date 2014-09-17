__author__ = 'LarsPfotzer'

from python_cle.robotsim.RobotInterface import Topic
import std_msgs.msg
import sensor_msgs.msg

class ROSComTest:
    float_test = Topic("/float_test", std_msgs.msg.Float32)

    image_test = Topic("/image_test", sensor_msgs.msg.Image)
