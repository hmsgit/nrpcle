Tutorial: Creating Metrics
==========================

In many experiments, it is desirable to record metrics on the simulation, e.g. in order to analyze
and visualize them after the experiment has been simulated. For this, the NRP uses the same
infrastructure as to interface with the simulated robot, i.e. it uses ROS topics.

As a consequence, to record metrics from a running simulation, simply add a transfer function that
targets a ROS topic, same as any other transfer function targeting the robot. An example can be found
in :doc:`neuron2robot`.

To record metrics on the robot, simply create a transfer function transferring from robot topics to
robot topics. Although there are no technical limitations for the names of topics to be used to
compute metrics, a clear separation of metrics topic from those used for the robot is beneficial
for the understandability of the transfer functions. We default naming convention of the neurorobotics
platform is to start topics for metrics with a */metrics* segment in the topic name.

Let us for example consider that we wanted to record the average speed of the robot. What we would
have to do is simply to create a transfer function to record the current velocity of the robot and
keep track of the total distance covered during the simulation. The latter can be done using transfer
function scoped variables, see :doc:`variables`.

.. note:: There is currently no direct support for variables from the BIBI XML

If we want to specify the average speed metric in Python, this amounts to the following code:

.. code-block:: python

    @nrp.MapVariable("s_total", initial_value=0)
    @nrp.MapRobotSubscriber("vel", "/husky/cmd_vel", geometry_msgs.msg.Twist)
    @nrp.Neuron2Robot(Topic("/metrics/avg_speed", std_msgs.msg.float))
    def linear_twist(t, s_total, vel, avg):
        if t = 0:
            pass
        s_total.value += vel.linear.x
        return s_total / t

.. note:: The parameter mapping decorators must appear before the *Neuron2Robot* decorator. Otherwise an exception will be thrown.