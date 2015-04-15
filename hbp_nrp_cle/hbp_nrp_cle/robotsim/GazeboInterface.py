"""
Interface for gzserver and gzbridge spawning classes.
"""

__author__ = 'Alessandro Ambrosano'


class IGazeboServerInstance(object):   # pragma: no cover
    """
    Takes care of starting a gzserver process somewhere, connect it to the given roscore.
    Each implementation has to take care of providing the methods start, stop and restart as
    well as a property containing the gzserver master URI.
    """

    def start(self, ros_master_uri):
        """
        Starts a gzserver instance connected to the local roscore (provided by
        ros_master_uri)

        :param: ros_master_uri The ros master uri where to connect gzserver.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def stop(self):
        """
        Stops the gzserver instance.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def get_gzserver_master_URI(self):
        """
        Returns a string containing the gzserver master
        URI (like:'http://bbpviz001.cscs.ch:11345')
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def restart(self):
        """
        Restarts the gzserver instance.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IGazeboBridgeInstance(object):
    """
    Takes care of starting a gzserver instance somewhere.
    """

    def start(self, gzserver_host, gzserver_port):
        """
        Starts the gzbridge instance represented by the object.

        :param gzserver_host The host where gzserver is running
        :param gzserver_port The port on which gzserver is running
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def stop(self):
        """
        Stops the gzbridge instance represented by the object.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def restart(self):
        """
        Restarts the gzbridge instance represented by the object.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
