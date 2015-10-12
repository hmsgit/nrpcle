Tutorial: Accessing virtual sensors
===================================

In many experiments, the simulated brain does not cover a full perception-cognition-action loop.
Rather, only an aspect of the brain is modeled. In many cases such as the visual cortex, the visualization
of the brain activity is rather easy and can be accomplished by raising a hand or equivalent tasks.

However, in some experiments a need arises to bypass functionality like image processing e.g. in
order to implement a reward when the robot has navigated to a certain spot. Here, an implementation
of the recognition of the robots position may inflate the validation of the navigation functionality.
On the other hand, the robot typically does not have a sensor to tell him the distance to a certain
object in the simulated environment since this is dependent on the environment.

The supported approach to solve such a problem is the introduction of virtual sensors. Such a virtual
sensor is not part of the robot itself, but provided by the experiment. However, it can be accessed
through transfer functions same as any other sensor.

Let us for example consider a virtual proximity sensor, available through the topic */husky/proximity*.
This sensor measures the robots distance to a certain object in the environment. The definition of
this object and the implementation of the sensor itself is subject to the experiment description. A tutorial
can be found in the documentation of the NRP Backend.

We want to implement a reward system for the neuronal network that will be implemented by issuing
spikes to a particular neuron if the distance measured by the proximity sensor is less than a predefined
threshold. In particular, the Poisson rate is increasing as the distance to the target spot decreases.
In this tutorial, we decide for a linear incline, though one could also chose an exponential increase,
depending on the neural implementation of the reward system in the brain model.

To implement this situation, we can simply create a transfer function from robot topic to neurons, same as
we do for real sensors of the robot such as the camera image (cf. :doc:`robot2neuron`).

The complete TF should look as follows:

.. code-block:: xml

  <transferFunction xsi:type="Robot2Neuron" name="proximity_reward">
    <device name="reward" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="0" to="3" step="2"/>
      <body xsi:type="Scale" factor="50.0">
        <inner xsi:type="Subtract">
          <operand xsi:type="Constant" value="2"/>
          <operand xsi:type="Min">
            <operand xsi:type="ArgumentReference" name="proximity"/>
            <operand xsi:type="Constant" value="2"/>
          </operand>
        </inner>
      </body>
    </device>
    <topic name="proximity" topic="/husky/proximity" type="std_msgs.msg.float"/>
  </transferFunction>

The equivalent Python Transfer Function then looks as follows:

.. code-block:: python

    @nrp.MapRobotSubscriber("proximity", Topic('/husky/proximity', std_msgs.msg.float))
    @nrp.MapSpikeSource("reward", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.Robot2Neuron()
    def proximity_reward(t, proximity, reward):
        reward.rate = 50.0 * (2.0 - min(proximity, 2.0))
