Tutorial: Accessing multiple neurons with device groups
=======================================================

In some applications, particularly when working with image data, it is not sufficient to work with single neurons.
This may work with the Braitenberg network because it only has a fixed amount of sensor neurons regardless of the image
resolution. However, this is different in the general case when an image should be processed by hundreds or thousands of neurons.
In this case, it is infeasible to create either hundreds of transfer functions that essentially do the same or have a single TF with hundreds of parameters,

For this reason, device groups have been invented. That is, a set of devices of the same kind (such as a thousand Poisson generators) can be accessed through a
device group. Such a device group always has the same properties as the original device type but returns numpy arrays instead of simple values.
Similarly, when an attribute is assigned a value and that value is a numpy array, the values are distributed to the devices accordingly. A device group
also accepts a single value to be assigned to a property. In that case, this value is assigned to the respective property for all devices.

To create a device group in Python, simply provide a list of neuron specifications instead of a single specification.

That means, instead of

.. code-block:: python

    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[0], nrp.leaky_integrator_alpha)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)

the following code will create a device group with both neurons:

.. code-block:: python

    @nrp.MapSpikeSink("wheel_neurons", [nrp.brain.actors[0], nrp.brain.actors[1]], nrp.leaky_integrator_alpha)

In the BIBI Configuration, there is an element *DeviceGroupChannel* so that the following specification of two device channels:

.. code-block:: xml

    <device name="left_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="0"/>
    </device>
    <device name="right_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="1"/>
    </device>


can be replaced by the following specification:


.. code-block:: xml

    <deviceGroup name="wheel_neurons" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="0"/>
      <neurons xsi:type="Index" population="actors" index="1"/>
    </deviceGroup>

.. warning::
    Currently, the support for device groups is not sufficiently tested.