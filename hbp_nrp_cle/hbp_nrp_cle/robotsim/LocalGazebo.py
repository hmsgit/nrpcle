"""
This module contains the classes needed to have all gazebo services running locally.
"""

from hbp_nrp_cle.robotsim.GazeboInterface import IGazeboServerInstance, IGazeboBridgeInstance
from hbp_nrp_cle.bibi_config.notificator import Notificator
import os

__author__ = 'Alessandro Ambrosano'


class LocalGazeboServerInstance(IGazeboServerInstance):
    """
    Represents a local instance of gzserver.
    """

    def start(self, ros_master_uri): # pylint: disable=unused-argument
        """
        Starts a gzserver instance connected to the local roscore (provided by
        ros_master_uri)

        :param: ros_master_uri The ros master uri where to connect gzserver.
        """
        Notificator.notify("Starting gzserver", False)
        os.system('/etc/init.d/gzserver start')

    def stop(self):
        """
        Stops the gzserver instance.
        """
        Notificator.notify("Stopping gzserver", False)
        os.system('/etc/init.d/gzserver stop')

    def restart(self, ros_master_uri):
        """
        Restarts the gzserver instance.
        """
        Notificator.notify("Restarting gzserver", False)
        os.system('/etc/init.d/gzserver restart')

    @property
    def gazebo_master_uri(self):
        """
        Returns a string containing the gazebo master
        URI (like:'http://bbpviz001.cscs.ch:11345')
        """
        return os.environ.get("GAZEBO_MASTER_URI")


class LocalGazeboBridgeInstance(IGazeboBridgeInstance):
    """
    Represents a local instance of gzbridge.
    """

    def start(self): # pylint: disable=unused-argument
        """
        Starts the gzbridge instance represented by the object.
        """
        Notificator.notify("Starting gzbridge", False)
        os.system('/etc/init.d/gzbridge start')

    def stop(self):
        """
        Stops the gzbridge instance represented by the object.
        """
        Notificator.notify("Stopping gzbridge", False)
        os.system('/etc/init.d/gzbridge stop')

    def restart(self): # pylint: disable=unused-argument
        """
        Restarts the gzbridge instance represented by the object.
        """
        Notificator.notify("Restarting gzbridge", False)
        os.system('/etc/init.d/gzbridge restart')
