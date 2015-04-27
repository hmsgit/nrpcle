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

    def start(self, ros_master_uri): # pylint: disable=unused-argument
        """
        Starts a gzserver instance connected to the local roscore (provided by
        ros_master_uri)

        :param: ros_master_uri The ros master uri where to connect gzserver.
        """
        os.system('/etc/init.d/gzserver start')

    def stop(self):
        """
        Stops the gzserver instance.
        """
        os.system('/etc/init.d/gzserver stop')

    def restart(self):
        """
        Restarts the gzserver instance.
        """
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

    def __init__(self):
        pass

    def start(self, gzserver_host, gzserver_port): # pylint: disable=unused-argument
        """
        Starts the gzbridge instance represented by the object.

        :param gzserver_host The host where gzserver is running
        :param gzserver_port The port on which gzserver is running
        """
        os.system('/etc/init.d/gzbridge start')

    def stop(self):
        """
        Stops the gzbridge instance represented by the object.
        """
        os.system('/etc/init.d/gzbridge stop')

    def restart(self, gzserver_host, gzserver_port): # pylint: disable=unused-argument
        """
        Restarts the gzbridge instance represented by the object.

        :param gzserver_host The host where gzserver is running
        :param gzserver_port The port on which gzserver is running
        """
        os.system('/etc/init.d/gzbridge restart')
