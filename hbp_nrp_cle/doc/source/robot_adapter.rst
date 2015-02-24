==============
Robot adapters
==============

Much like the :doc:`brain_adapter`, the CLE knows two kinds of robot adapters, communication and control adapters.
They also serve mainly the same purposes.

The CLE assumes communication with the robot simulation via topics, i.e. pairs of a name and type that uniquely identify some property of the robot.
These topics can either be subscribed or published to. A subscribed topic may receive a new value at each timestep and knows whether its value has changed since the last timestep. A published topic can be written to.

.. note::

    So far, the definition does support asynchronous communication such as topic communication in ROS, but it does not imply it. Robot communication adapters have the chance to refresh buffers.
    In the standard ROS implementation of the robot communication adapter, this is used to reset the changed flags to False but it could also be used to implement a blocking service-based communication paradigm.

Much like brain devices, robot channels (subscribers or publishers) can get additional configuration that is specific to the robot communication adapter. Of course, also for the robot adapters, the communication adapter has to fit to the control adapter although this is not enforced for distribution reasons.

The CLE implementation contains two sets of robot adapters that is the ROS/Gazebo adapters and the mock adapters. Here, the communication adapter only has a dependency to ROS whereas the control adapter has an additional dependency to a patched version of the Gazebo plugin that exposes Gazebos internal state through ROS.
Particularly interesting, if we eventually wish to exchange the simulated robot by a real world robot, we can stick to the ROSCommunicationAdapter. However, it will be very hard to write an adapter that is truly able to "control the simulation" i.e. the real world and if we really manage to do that, we are not going to publish it. Instead, such a control adapter would hope that the neuronal simulation runs fast enough to be reasonable but of course do not control anything.