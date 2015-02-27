"""
CLE states exposed via ROS. This class is only about constants.
"""


class ROSCLEState(object):
    """
    Describe the state that are exposed to the outside world through ROS.
    """
    STARTED = "started"
    STOPPED = "stopped"
    INITIALIZED = "initialized"
    PAUSED = "paused"
