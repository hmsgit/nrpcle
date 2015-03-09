Tutorial: Monitoring neurons
============================

To monitor neuronal network activity, neuronal network information is published to a robot topic that is used for display purposes instead of
being used by the robot. Thus, monitoring information can be specified using the normal TF mechanisms.

However, since it is a very common task, the BIBI Configuration also provides a convenient shortcut for such monitoring tasks, namely a special
*Neuron2Robot* TF called **Neuron2Monitor**. To specify a monitor with this shortcut, users only have to insert the following code to monitor the activity
of a neuron:

.. code-block:: xml

    <transferFunction xsi:type="Neuron2Monitor" name="left_wheel_neuron_monitor">
      <device name="left_wheel_neuron" type="PopulationRate">
        <neurons xsi:type="Index" population="actors" index="0"/>
      </device>
    </transferFunction>

This specification already suffices to create a TF that will take the population rate of the neuron responsible
for the left wheel motor and send it to a special monitoring topic that is used by the frontend.

Unlike other TFs, monitoring TFs may only specify exactly one device channel and any body if present is ignored. However, it is possible to connect
the device channel with multiple neurons.