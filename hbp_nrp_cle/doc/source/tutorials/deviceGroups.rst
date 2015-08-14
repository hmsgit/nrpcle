Tutorial: Accessing multiple neurons with device groups
=======================================================

In some applications, particularly when working with image data, it is not sufficient to work with single neurons.
This may work with the simplified Braitenberg network because it only has a fixed amount of sensor neurons regardless of the image
resolution. However, this is different in the general case when an image should be processed by hundreds or thousands of neurons.
In this case, it is infeasible to create either hundreds of transfer functions that essentially do the same or have a single TF with hundreds of parameters.

For this reason, device groups have been created. That is, a set of devices of the same kind (such as a thousand Poisson generators) can be accessed through a
device group. Such a device group always has the same properties as the original device type but returns numpy arrays instead of simple values.
Similarly, when an attribute is assigned a value and that value is a numpy array, the values are distributed to the devices accordingly. A device group
also accepts a single value to be assigned to a property. In that case, this value is assigned to the respective property for all devices.

To create a device group in Python, two ways are supported. You can either map a population of neurons to devices to attach them to, or create chains of neurons.
Alternatively, you can use a combination of both.

That means, instead of

.. code-block:: python

    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[0], nrp.leaky_integrator_alpha)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)

the following code will create a device group with both neurons:

.. code-block:: python

    @nrp.MapSpikeSink("wheel_neurons", nrp.chain_neurons(nrp.brain.actors[0], nrp.brain.actors[1]), nrp.leaky_integrator_alpha)

What this assignment will do is that it tells the TF Framework that the parameter *wheel_neurons* should be mapped to an array of leaky integrators.
This array should stack a leaky integrator for *nrp.brain.actors[0]* and *nrp.brain.actors[1]*. Within the function, the devices group allows
two different modes of interaction, either by accessing the devices by array indices or by directly accessing the properties from the device group.
In that case, the device group automatically collects the properties from the devices into a numpy array.

.. code-block:: python

    # accessing the device for the left wheel by index
    # left_wheel_voltage is now a double
    left_wheel_voltage = wheel_neurons[0].voltage
    # accessing the device voltages from all devices in the device group
    # wheel_voltage is now a numpy array
    wheel_voltages = wheel_neurons.voltage
    # since the TF framework caches the voltages, they are equal
    assert left_wheel_voltage == wheel_voltages[0]

Conversely, the devices of a device group can also be configured together by passing a numpy array as argument with the same length as there are devices in the device group.
Alternatively, it is allowed to specify a single value. In this case, this value is passed to all devices in this device group.

.. code-block:: python

    @nrp.MapSpikeSource("neurons", nrp.map_neurons(range(0,5), lambda i: nrp.brain.sensors[i]), nrp.poisson_generator)
    def test_device_groups(t, neurons):
        # We can assign the rates of each poisson generator individually
        neurons[0].rate = 42.0
        # Alternatively, we can configure all devices at once using a numpy array
        neurons.rate = numpy.array([1.0, 1.1, 1.2, 1.3, 1.4])
        # Instead of a numpy array, we could also have used anything that is indexable as well, such as a list
        neurons.rate = [1.0, 1.1, 1.2, 1.3, 1.4]
        # If we wanted to set the rates for all devices to an equal rate, we can also use a single value
        # This will set the rates to 42.0 for all devices
        neurons.rate = 42.0

On the other hand, in many cases the size of a population to connect to is too large or even unknown when writing the transfer functions.
In that case, we can map a collection such as a range to neuron connections to fulfill the same goal.

.. code-block:: python

    @nrp.MapSpikeSink("wheel_neurons", nrp.map_neurons(range(0,2), lambda i: nrp.brain.actors[i]), nrp.leaky_integrator_alpha)

The intention behind this assignment is that the function *nrp.map_neurons* maps an index set to a list of neurons the neuronal simulation should connect to.
The lambda expression may either return a single neuron specification or a list. In the latter case, the resulting lists get concatenated so that the function
can be used to specify patterns.

In the BIBI Configuration, there is an element *DeviceGroupChannel* to represent device group accesses. This channel can be connected to neurons using *NeuronGroupSelector*
much like the normal devices are connected by *NeuronSelector*\s. Currently, we support chains and maps such as described above.

As a result, the following specification of two device channels

.. code-block:: xml

    <device name="left_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="0"/>
    </device>
    <device name="right_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="1"/>
    </device>


can be replaced by the following specification using the *ChainSelector* element:

.. code-block:: xml

    <deviceGroup name="wheel_neurons" type="LeakyIntegratorAlpha">
      <connector xsi:type="ChainSelector">
        <neurons xsi:type="Index" population="actors" index="0"/>
        <neurons xsi:type="Index" population="actors" index="1"/>
      </connector>
    </deviceGroup>

This model resembles exactly the first option of creating a chain of neuron specifications.

Chains may not only consist of neurons but also of other neuron group selectors such as other chains or mappings. Mappings are supported by the BIBI Model through
*MapSelector* elements that resemble the mapping operations presented above.

To reflect the degrees of freedom in the way how an index is mapped to neuron indices, the map connector uses *NeuronSelectorTemplate* elements. These templates allow to specify
which neurons can be connected by arithmetic index operations. These index operations are not modeled in detail but only given as text. In particular, the template elements
allow passing in a string that may contain the letter *i* which is then replaced by the respective index.

The specification of mappings is presented in the following Listing, representing the same situation specified with a mapping.

.. code-block:: xml

    <deviceGroup name="wheel_neurons" type="LeakyIntegratorAlpha">
      <connector xsi:type="MapSelector">
        <source xsi:type="Range" from="0" to="2" population="actors">
        <pattern xsi:type="IndexTemplate" index="i"/>
      </connector>
    </deviceGroup>

Unlike the Python DSL, the BIBI Model only allows that mappings originate from the same population. It is not possible to specify multiple populations to connect to within
a single mapping connector. The population from which the neurons are drawn is given by the source element of the mapping element. Here, we currently allow ranges, lists and
entire populations. However, for a population it is also required to specify the amount of neurons created in this population.

.. warning::
    Currently, the support for device groups is under development and may not be sufficiently tested.
