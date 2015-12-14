Tutorial: Writing a Neuron2Robot TF
===================================

This tutorial assumes you have familiarized yourself with the Braitenberg experiment. If not, please
consider reading the :doc:`setup` first.

Transfer Functions may be specified in two ways. The first way is to specify it on a high level
through the :doc:`../architecture/BIBI-configuration`. Alternatively, one may specify TFs in Python
through the :doc:`../codedoc/hbp_nrp_cle.tf_framework`. If specified using the BIBI configuration,
the TFs will be converted to Python code through the :doc:`../architecture/BIBI-configuration-generator`.

.. note::
    Currently, one can only specify the BIBI Configuration files through manually writing XML. However, it is intended for the future to be able to specify them in
    a graphical language as e.g. through a (web-based) editor as well.

Specification in a BIBI Configuration XML file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note:: As there is a graphical editor planned, this section is subject to change as soon as the editor gets available.

We will specify the TF for the Husky to transmit the actor neuron voltages to commands to the Husky
robot. This direction is usually the easier one.

A new TF
--------

We start by defining a TF with a name.

.. code-block:: xml

    <transferFunction xsi:type="Neuron2Robot" name="linear_twist">
    </transferFunction>

The XML fragment tells that the TF is a Neuron2Robot TF named linear twist. So far, this TF is not
valid because it does not connect anything.

Connecting to neurons
---------------------

We begin by connecting the TF to the actor neurons.

.. code-block:: xml

    <device name="left_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="0"/>
    </device>
    <device name="right_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="1"/>
    </device>

Here, we defined that we need the voltage of the actor neuron 0 (which is neuron 6) as
*left_wheel_neuron* and the voltage of the actor neuron 1 (which is neuron 7) as *right_wheel_neuron*.
The type *LeakyIntegratorAlpha* is a simple voltmeter that only measures the voltage of this neuron.

.. note::
    The declaration is not specific to whether the voltmeter is implemented through an artificial
    neuron or through taking the voltage of an existing neuron. In particular, this is an
    implementation detail of the brain communication adapter.

Connecting to the robot
-----------------------

The connection to the robot topic is a bit more complex since we need to assign the robot topic an
expression of what shall be sent to that robot topic. We do this by the specification of a topic
channel. We can either create a new channel or use the default return value channel. For the XML,
the difference is not really important, though the generated code is more readable if the return
value channel is used if not already occupied.

We do this by the following XML:

.. code-block:: xml

    <returnValue name="wheel" topic="/husky/cmd_vel" type="geometry_msgs.msg.Twist">
      <body xsi:type="Call" type="geometry_msgs.msg.Twist">
      </body>
    </returnValue>

This will render as a function where the return value is sent to the **/husky/cmd_vel** topic.
Remember that we use ROS for our robot simulation and thus we have to connect the TF to the ROS
topic that represents the robots velocity. Alternatively, we can create a dedicated topic channel
by the following XML:

.. code-block:: xml

    <topic name="wheel" topic="/husky/cmd_vel" type="geometry_msgs.msg.Twist">
      <body xsi:type="Call" type="geometry_msgs.msg.Twist">
      </body>
    </topic>

The difference between these options is minimal. A return channel renders more nicely and has a
presumably slightly better performance but you may only create a single return channel. All of the
next topic channels must be created as dedicated topic channels.

Transferring data
-----------------

Either way, the body of the channel (which tells the CLE that this topic is written to) is still
almost empty. In the example, we have filled it with a *Call* expression to *geometry_msgs.msg.Twist*.
At the moment, the twist message does not get any parameters. So let us provide the parameters.

A Twist message consists of a linear and an angular part. The actor neurons describe the voltages
sent to the left and right motors. Thus, we simply take the minimum of these voltages to get the
linear part of the Twist. This settles in the following XML code:

.. code-block:: xml

    <argument name="linear">
      <value xsi:type="Call" type="geometry_msgs.msg.Vector3">
        <argument name="x">
          <value xsi:type="Scale" factor="20">
            <inner xsi:type="Min">
              <operand xsi:type="ArgumentReference" name="left_wheel_neuron" property="voltage"/>
              <operand xsi:type="ArgumentReference" name="right_wheel_neuron" property="voltage"/>
            </inner>
          </value>
        </argument>
        <argument name="y">
          <value xsi:type="Constant" value="0"/>
        </argument>
        <argument name="z">
          <value xsi:type="Constant" value="0"/>
        </argument>
      </value>
    </argument>

Here, we specified that the x component of the linear vector is the minimum of both voltages scaled
by a factor of 20. Similarly, the angular component of the twist is specified as the subtraction of
both motor voltages scaled by a constant factor:

.. code-block:: xml

    <argument name="angular">
      <value xsi:type="Call" type="geometry_msgs.msg.Vector3">
        <argument name="x">
          <value xsi:type="Constant" value="0"/>
        </argument>
        <argument name="y">
          <value xsi:type="Constant" value="0"/>
        </argument>
        <argument name="z">
          <value xsi:type="Scale" factor="100">
            <inner xsi:type="Subtract">
              <operand xsi:type="ArgumentReference" name="right_wheel_neuron" property="voltage"/>
              <operand xsi:type="ArgumentReference" name="left_wheel_neuron" property="voltage"/>
            </inner>
          </value>
        </argument>
      </value>
    </argument>

For the next steps, you may either see what Python code is generated for such a *Neuron2Robot* TF or
you may want to jump to the specification of a *Robot2Neuron* TF in :doc:`robot2neuron`.

Putting it together
-------------------

The complete TF should look as follows:

.. code-block:: xml

  <transferFunction xsi:type="Neuron2Robot" name="linear_twist">
    <device name="left_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="1"/>
    </device>
    <device name="right_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="2"/>
    </device>
    <returnValue name="wheel" topic="/husky/cmd_vel" type="geometry_msgs.msg.Twist">
      <body xsi:type="Call" type="geometry_msgs.msg.Twist">
        <argument name="linear">
          <value xsi:type="Call" type="geometry_msgs.msg.Vector3">
            <argument name="x">
              <value xsi:type="Scale" factor="20">
                <inner xsi:type="Min">
                  <operand xsi:type="ArgumentReference" name="left_wheel_neuron" property="voltage"/>
                  <operand xsi:type="ArgumentReference" name="right_wheel_neuron" property="voltage"/>
                </inner>
              </value>
            </argument>
            <argument name="y">
              <value xsi:type="Constant" value="0"/>
            </argument>
            <argument name="z">
              <value xsi:type="Constant" value="0"/>
            </argument>
          </value>
        </argument>
        <argument name="angular">
          <value xsi:type="Call" type="geometry_msgs.msg.Vector3">
            <argument name="x">
              <value xsi:type="Constant" value="0"/>
            </argument>
            <argument name="y">
              <value xsi:type="Constant" value="0"/>
            </argument>
            <argument name="z">
              <value xsi:type="Scale" factor="100">
                <inner xsi:type="Subtract">
                  <operand xsi:type="ArgumentReference" name="right_wheel_neuron" property="voltage"/>
                  <operand xsi:type="ArgumentReference" name="left_wheel_neuron" property="voltage"/>
                </inner>
              </value>
            </argument>
          </value>
        </argument>
      </body>
    </returnValue>
  </transferFunction>

Specification in Python
^^^^^^^^^^^^^^^^^^^^^^^

A TF in Python is basically a Python function with a set of decorators. These decorators create a TF
from a simple Python function by specifying where the function parameters come from and what should happen
with the functions return value. Let us begin to manually implement the TF from above in Python code.

.. note:: The following code will usually be generated by the :doc:`../architecture/BIBI-configuration-generator` if BIBI Configurations are used.

A (not so) new TF
-----------------

.. code-block:: python

    import hbp_nrp_cle as nrp

    @nrp.Neuron2Robot()
    def linear_twist(t):
        pass

This code already creates a TF named **linear_twist** as a *Neuron2Robot* TF.

Connecting to the neuronal network
----------------------------------

We access the neuronal network through parameters of the TF function. For this, we need to introduce
a new parameter and have to connect it to the brain accordingly. This connection is again done
through a decorator. This decorator takes as inputs

1. The name of the parameter that should be connected
2. The neurons that should be connected
3. The device type that should be created
4. Additional device configuration

The specification of the neurons that can be connected works through a specification starting from
**nrp.brain**. Since TFs exist independently from the brain instance, the object accessible through
nrp.brain records all the steps and thus represents a function that will when given a brain instance
select the neurons that should be connected to the TF.

The device types are the device types supported by the CLE. In particular, the following are allowed:

* nrp.leaky_integrator_alpha = :class:`hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha`
* nrp.leaky_integrator_exp = :class:`hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp`
* nrp.fixed_frequency = :class:`hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator`
* nrp.poisson = :class:`hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator`
* nrp.detector = :class:`hbp_nrp_cle.brainsim.BrainInterface.ISpikeDetector`
* nrp.dc_source = :class:`hbp_nrp_cle.brainsim.BrainInterface.IDCSource`
* nrp.ac_source = :class:`hbp_nrp_cle.brainsim.BrainInterface.IACSource`
* nrp.nc_source = :class:`hbp_nrp_cle.brainsim.BrainInterface.INCSource`
* nrp.population_rate = :class:`hbp_nrp_cle.brainsim.BrainInterface.IPopulationRate`

Of course, not all device types are suitable for reading purposes.

If we want to specify the devices like above, this amounts to the following Python code:

.. code-block:: python

    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[0], nrp.leaky_integrator_alpha)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
    @nrp.Neuron2Robot()
    def linear_twist(t, left_wheel_neuron, right_wheel_neuron):
        pass

.. note:: The parameter mapping decorators must appear before the *Neuron2Robot* decorator. Otherwise
an exception will be thrown.

The rationale behind the naming *MapSpikeSink* is that the generated devices are effectively sinks as
they consume spikes.

Although we have specified how the TF can be connected to a neuronal simulator, we have not yet
decided on which neuronal simulator to choose. Moreover, it is perfectly valid to use a mock neuronal
simulator as e.g. for unit testing of the TF.

In the last code snippet, we have not used additional device configuration. Such additional device
configuration is specific to a particular neuronal simulator and may be used for various purposes
but as the data is transferred in the TF anyhow, this is usually not so important as similar effects
can be gained more easily by varying scale factors.

Connecting to the robot
-----------------------

Of course, so far there is nothing to unit test since the TF is not yet doing anything. To change
this, we have to assign a robot topic channel. The most convenient form is to simply capture the
methods return value and send the output to a robot topic. To do this, we simply need to add an
argument to the *@Neuron2Robot* decorator as shown below:

.. code-block:: python

    @nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))

Now, we only need to ensure that we return something that is not *None* but an instance of *geometry_msgs.msg.Twist*.

As the next step, we learn how to specify a TF in the opposite direction: :doc:`robot2neuron`.