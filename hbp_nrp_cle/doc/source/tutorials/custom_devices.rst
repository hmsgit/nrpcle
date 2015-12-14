Tutorial: A custom device for reusable image processing
=======================================================

Some applications are recurrent and could be reused among multiple applications. A typical
application here is image recognition, i.e. how to *map* an image from the robot (typically in RGB)
to a neuronal network. We assume that the connection to the network is to some degree always similar
and thus one would want to create a common denominator to specify such a connection only once and
reuse it in multiple applications.

Custom devices provide a way for such a reusable connection.

Custom devices are devices with a custom algorithm to bind to a neuronal network. Unlike other
devices that are connected to a neuronal network through directly by the brain communication adapter,
custom devices provide their own logic how they connect to a given set of neurons.

.. note:: We have not really used custom devices so far at this stage of the NRP platform. We will
add more documentation here as soon as we have done so.