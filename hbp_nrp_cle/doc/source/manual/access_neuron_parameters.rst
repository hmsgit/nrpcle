Accessing neural network variables to configure Transfer Functions
==================================================================

In some cases, it may be beneficial to access parameters of the neural network to configure
Transfer Functions. These variables may be specified as global variables of the neural network
module that is to be loaded. These variables may contain size information of a dynamically sizable
(typically engineered) neural network.

There are two conceptually different ways to access neural network variables.

Accessing neural network variables in the body of a Transfer Function
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Accessing neural network parameters in the body of a Transfer Function is quite simple through the
**nrp.config.brain_root** variable. This variable allows direct access to the Python module used as
neural network (in case of a Python network) or the generated network object in case a H5 network is
used.

Accessing neural network variables to configure Transfer Functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The problem when accessing variables of the neural network in the specification of a Transfer Function
(i.e., in the decorators) is that the value of these variables
may change when the neural network is changed. However, the specification of a Transfer Function is
cached, i.e. rather than re-executing the code of a Transfer Function for example on a reset, we execute
required operations on parts of Transfer Functions. This allows us to do partial resets, for example to keep
recorded csv data when the brain is changed during a simulation.

Therefore, the **nrp.config.brain_root** variable must **not** be used to configure Transfer Function devices.
The CLE simply would not recognize any changes to the network variable.

Instead, such a variable access can be implemented in a very similar fashion to the access of neural network
populations, as these also change whenever the brain is reloaded. That is, one simply has to access
the variable through the provided API **nrp.brain**. This variable tracks its access and thus allows the NRP
to track that the configuration depends on the neural network variable and to recompute the current value in case
the neural network changed during the simulation.

For an example, consider that you have two variables in your network called *ImageNumPixelRows* and
*ImageNumPixelColumns* that correspond to the resolution of an image. Based on these variables, a
device group has to be used in a Transfer Function that contains as many devices as there are pixels
in the converted image. In this situation, one would typically want to easily change this image
resolution without having to adjust several magic numbers in all affected Transfer Functions.

To solve this, the supported way is to specify the access to neural network variables through the
**nrp.brain** variable. You can simply do arithmetic calculations, navigate
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
we offer a way to insert a blackbox in this procedure where you can define a lazy evaluation function yourself.
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