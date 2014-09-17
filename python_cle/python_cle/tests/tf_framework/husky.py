__author__ = 'GeorgHinkel'

from python_cle.robotsim.RobotInterface import Topic


class Husky:
    class RightArm:
        pose = Topic("/husky1/joint325/pose", float)

    class LeftArm:
        twist = Topic("/husky1/leftArm/twist", float)

    class Eye:
        camera = Topic("/husky1/sensors/camera1", list)