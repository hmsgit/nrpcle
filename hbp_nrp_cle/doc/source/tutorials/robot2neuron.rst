Tutorial: Writing a Robot2Neuron TF
===================================

This tutorial assumes you have familiarized yourself with the Braitenberg experiment. If not, please consider reading the :doc:`setup` first.

Transfer Functions may be specified in two ways. The first way is to specify it on a high level through the :doc:`../BIBI-configuration`. Alternatively,
one may specify TFs in Python through the :doc:`../hbp_nrp_cle.tf_framework`. If specified using the BIBI configuration, the TFs will be converted to Python code
through the :doc:`../BIBI-configuration-generator`.

.. note::
    Currently, one can only specify the BIBI Configuration files through manually writing XML. However, it is intended for the future to be able to specify them in
    a graphical language as e.g. through a (web-based) editor as well.

Specification in a BIBI Configuration XML file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note:: As there is a graphical editor planned, this section is subject to change as soon as the editor gets available.

Again a new TF
--------------

The definition of a *Robot2Neuron* TF is very similar to a *Neuron2Robot* TF. We start by the following XML code:

.. code-block:: xml

    <transferFunction xsi:type="Robot2Neuron" name="eye_sensor_transmit">
    </transferFunction>

This will create a TF called *eye_sensor_transmit*.

Connecting to the robot simulation
----------------------------------

Connecting a Robot2Neuron TF to a robot simulation is pretty straightforward as we only have to specify the topic that should be listened to.

.. code-block:: xml

    <transferFunction xsi:type="Robot2Neuron" name="eye_sensor_transmit">
      <topic name="camera" topic="/husky/camera" type="sensor_msgs.msg.Image"/>
    </transferFunction>

Connecting to the neuronal network
----------------------------------

To connect a TF to the neuronal network, we first need to specify which neurons we want to connect with. Here, we have three channels, one for the
right red pixels, one for the left right pixels and one for the green and blue pixels.

.. code-block:: xml

    <device name="red_left_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="0" to="3" step="2"/>
    </device>
    <device name="red_right_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="1" to="4" step="2"/>
    </device>
    <device name="green_blue_eye" type="Poisson">
      <neurons xsi:type="Index" population="sensors" index="4"/>
    </device>

Here, the red right pixels should be connected to neurons *0* and *2*, the left red pixels should be connected to neurons *1* and *3* and finally the green and blue pixels
should be connected to neuron *4*. We realized the notion of multiple neurons with a range directive, but we also support a List that is more general. All of these devices shall be realized as Poisson generators.

Transferring data
-----------------

Now comes the tricky part since we need to transfer the image data to the devices. Here we assume to have a suitable library function.
The rationale here is that TFs should be restricted to very simple transformations
except for a few more advanced transformations that are purely implemented in Python. In our case, there is a library function for the image recognition of the Braitenberg
experiment in *tf_lib.detect_red*.

The configuration options may get more expressive in the future but right now it is limited to simple arithmetics. However, the arithmetics still allow us to calibrate the TF by
scaling input and output values.

However, we need to ensure that an image is only traversed once and the results are used for all of the Poisson generators.
We do this by using local variables which we then use to specify the body of the device channels.

.. code-block:: xml

    <local name="image_results">
      <body xsi:type="Call" type="hbp_nrp_cle.tf_framework.tf_lib.detect_red">
        <argument name="image">
          <value xsi:type="ArgumentReference" name="camera" property="value"/>
        </argument>
      </body>
    </local>

We can then implement the device channel bodies using this local variable. Here, we have to know that the library function that we use returns an object with
three attributes, one for the ratio of red pixels in the left half image, one with the ratio of red pixels in the right half image and one with the ratio non-red pixels in the
image overall. The distinction whether a pixel is red is done in the library since the neuronal network with its effectively three sensor neurons is not capable of a reliable
understanding of what a red color is which is why it is not useful to scale down the camera image to only two pixels.
In particular, we decide whether a given color is red based on the HSV color model.

.. code-block:: xml

    <device name="red_left_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="0" to="3" step="2"/>
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="left"/>
      </body>
    </device>
    <device name="red_right_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="1" to="4" step="2"/>
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="right"/>
      </body>
    </device>
    <device name="green_blue_eye" type="Poisson">
      <neurons xsi:type="Index" population="sensors" index="4"/>
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="go_on"/>
      </body>
    </device>

Putting it all together
-----------------------

The complete TF should look as follows:

.. code-block:: xml

  <transferFunction xsi:type="Robot2Neuron" name="eye_sensor_transmit">
    <local name="image_results">
      <body xsi:type="Call" type="hbp_nrp_cle.tf_framework.tf_lib.detect_red">
        <argument name="image">
          <value xsi:type="ArgumentReference" name="camera" property="value"/>
        </argument>
      </body>
    </local>
    <device name="red_left_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="0" to="3" step="2"/>
      <!--body xsi:type="Scale" factor="0.002"-->
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="left"/>
      </body>
    </device>
    <device name="red_right_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="1" to="4" step="2"/>
      <!--body xsi:type="Scale" factor="0.002"-->
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="right"/>
      </body>
    </device>
    <device name="green_blue_eye" type="Poisson">
      <neurons xsi:type="Index" population="sensors" index="4"/>
      <!--body xsi:type="Scale" factor="0.00025"-->
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="go_on"/>
      </body>
    </device>
    <topic name="camera" topic="/husky/camera" type="sensor_msgs.msg.Image"/>
  </transferFunction>

Specification in Python
^^^^^^^^^^^^^^^^^^^^^^^

Same as for *Neuron2Robot*, a *Robot2Neuron* TF in Python is basically a Python function with a set of decorators. These decorators create a TF from a simple Python function by specifying where the function parameters come from and what should happen
with the functions return value. Let us begin to manually implement the TF from above in Python code.

.. note:: The following code will usually be generated by the :doc:`../BIBI-configuration-generator` if BIBI Configurations are used.

Eye_sensor_transmit in Python
-----------------------------

Hardly surprising, the declaration of a *Robot2Neuron* TF in Python looks very similar to the specification of a *Neuron2Robot* TF.

.. code-block:: python

    import hbp_nrp_cle.tf_framework as nrp

    @nrp.Robot2Neuron()
    def eye_sensor_transmit(t):
        pass

This will define a new *Robot2Neuron* TF and add it to the default TF manager instance.

Connecting to the robot simulation
----------------------------------

Similarly, the connection to the robot simulation is again done through a mapping decorator as follows:

.. code-block:: python

    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.Robot2Neuron()
    def eye_sensor_transmit(t, camera):

The decoration tells the CLE that the *camera* parameter originates from a robot topic with the arguments
as provided. The *camera* parameter will now be a robot subscriber that provides two properties: The *value* which is the last received
image and *changed* which indicates whether the value has changed since the last simulated step.

Connecting to the neuronal network
----------------------------------

As we now have three different neuron groups, we do not use the return channel but use dedicated channels for the devices. That is, we use dedicated parameters and decorators as follows:

.. code-block:: python

    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
    @nrp.Robot2Neuron()
    def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):

This has the same effect as the XML from above except that in Python implementation, we are not limited to use library functions but are free to implement the color detection directly.
Thus, the Python way is more flexible but in the long term we aim to provide a better tool support through a graphical editor. However, also pure Python TFs will be supported, see :doc:`python_only_tfs`.