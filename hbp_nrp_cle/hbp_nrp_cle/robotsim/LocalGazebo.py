"""
This module contains the classes needed to have all gazebo services running locally.
"""

from hbp_nrp_cle.robotsim.GazeboInterface import IGazeboServerInstance, IGazeboBridgeInstance
import os

__author__ = 'Alessandro Ambrosano'


class LocalGazeboServerInstance(IGazeboServerInstance):
    """
    Represents a local instance of gzserver.
    """

    def __init__(self):
        pass

    def start(self, ros_master_uri): # pylint: disable=W0613
        os.system('/etc/init.d/gzserver start')

    def stop(self):
        os.system('/etc/init.d/gzserver stop')

    def restart(self):
        os.system('/etc/init.d/gzserver restart')

    @property
    def gazebo_master_uri(self):
        return os.environ.get("GAZEBO_MASTER_URI")


class LocalGazeboBridgeInstance(IGazeboBridgeInstance):
    """
    Represents a local instance of gzbridge.
    """

    def __init__(self):
        pass

    def start(self, host, port): # pylint: disable=W0613
        os.system('/etc/init.d/gzbridge start')

    def stop(self):
        os.system('/etc/init.d/gzbridge stop')

    def restart(self):
        os.system('/etc/init.d/gzbridge restart')
