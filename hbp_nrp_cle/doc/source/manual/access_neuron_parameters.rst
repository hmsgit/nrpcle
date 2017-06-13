Accessing neural network variables to configure Transfer Functions
==================================================================

In some cases, it may be beneficial to access parameters of the neural network to configure
Transfer Functions. These variables may be specified as global variables of the neural network
module that is to be loaded. These variables may contain size information of a dynamically sizable
(typically engineered) neural network.

The problem when accessing variables of the neural network is that the value of these variables
may change when the neural network is changed.

Such a variable access can be implemented in a very similar fashion to the access of neural network
populations, as these also change whenever the brain is reloaded. That is, one simply has to access
the variable through the provided API **nrp.brain**.

For an example, consider that you have two variables in your network called *ImageNumPixelRows* and
*ImageNumPixelColumns* that correspond to the resolution of an image. Based on these variables, a
device group has to be used in a Transfer Function that contains as many devices as there are pixels
in the converted image. In the body of a Transfer Function, one can query the current value of the
variables through the **nrp.config.brain_root** variable, but there is no this variable is initialized
when the Transfer Function is created.

One would be able to easily
change this image resolution without having to adjust several magic numbers in all affected Transfer
Functions, but changing this number during a running simulation should have an effect to the
Transfer Functions accessing this parameter.

The problem here is that a change of the neural network implies that all neural network devices are
recreated, but they are recreated based on the original specification that is kept in memory. This
way, the Transfer Functions do not lose state that is not affected by the changed neural network
such as the state of variables or the last recorded images.

To solve this, the supported way is to specify the access to neural network variables in a similar way
to the access of populations (as the object identity of populations also change when the brain is
reloaded), namely through the **nrp.brain** variable. You can simply do arithmetic calculations, navigate
through the network model using attribute or index accesses or call methods. However, some parts of
the Python language such as the *in* operator or list comprehension are not supported.
The usage is exemplified in the following listing.

.. code-block:: python

    @nrp.MapSpikeSource("LGNBrightInput", nrp.map_neurons(nrp.nrange(0, nrp.brain.ImageNumPixelRows * nrp.brain.ImageNumPixelColumns), lambda i: nrp.brain.LGNBright[i]), nrp.dc_source)
    @nrp.Robot2Neuron()
    def my_transfer_function(t, LGNBrightInput):
        ...

The way how this works is that any access to the **nrp.brain** variable implicitly defines a function that is
executed lazily when the neural network is loaded or changed.

A consequence of this is that only very few functions are supported by this type of API. This is also
the reason that the regular *range* function can not be used if the arguments depend on the neural network
and a custom function **nrp.nrange** has to be used that extends the functionality of the builtin range
by a support of this lazy evaluation API.

If a use case requires a complicated access beyond the currently supported subset of the Python language,
we offer a way to insert a blackboy in this procedure where you can define a lazy evaluation function yourself.
This is done through the **nrp.resolve** method. You have to supply a function to this method that either
takes no arguments, one argument that is the readily loaded neural network or two arguments consisting of
the neural network module and the brain communication adapter.

The usage is depicted in the following listing:

.. code-block:: python

    @nrp.MapSpikeSource("LGNBrightInput", nrp.map_neurons(nrp.nrange(0, nrp.resolve(lambda brain: brain.ImageNumPixelRows * brain.ImageNumPixelColumns)), lambda i: nrp.brain.LGNBright[i]), nrp.dc_source)
    @nrp.Robot2Neuron()
    def my_transfer_function(t, LGNBrightInput):
        ...

However, we believe that where possible, the first type of this specification is more understandable and therefore encourage its usage.