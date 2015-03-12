"""
The Husky robot model used by milestone 2
"""

from hbp_nrp_cle.robotsim.RobotInterface import Topic
import geometry_msgs.msg
import sensor_msgs.msg

__author__ = 'GeorgHinkel'

wheel_twist = Topic("/husky/cmd_vel", geometry_msgs.msg.Twist)
eye_sensor = Topic("/husky/camera", sensor_msgs.msg.Image)