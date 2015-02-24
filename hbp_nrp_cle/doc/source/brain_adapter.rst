==============
Brain adapters
==============

There are two kinds of brain adapters in the CLE, communication adapters and control adapters.
While a communication adapter organizes how data is transmitted to the neuronal simulation, the
control adapter specifies how to control the neuronal simulation. Of course, the adapters used within
a simulation should match together, i.e. reference the same neuronal simulator. However, this is not
enforced by the CLE in order to leave options to deploy the communication adapters and control adapters
on different machines.

Currently, we do have two isolated brain adapters, one for NEST through the PyNN interface and to be
used as a mock. While the NEST adapter is primarily used in the production system, the mock adapter
is mainly used for testing.

Commonalities of all brain adapters
-----------------------------------

In general, a brain communication adapter must implement the interface :class:`hbp_nrp_cle.brainsim.BrainInterface.IBrainCommunicationAdapter` and a brain control adapter must implement the interface :class:`hbp_nrp_cle.brainsim.BrainInterface.IBrainControlAdapter`. These classes are considered interfaces because their name starts with the letter *I* and all their members throw an exception that they are not implemented.

While the control adapter interface is straight forward, there are some points to remark for the communication adapter, particularly for the methods *register_spike_sink* and *register_spike_source*. The supported device types are:

For *register_spike_sink* that registers a devices that consumes spikes:

* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.IPopulationRate`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.ISpikeRecorder`
* An instance of :class:`hbp_nrp_cle.brainsim.BrainInterface.ICustomDevice`.

For the control adapter, allowed device types are:

* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.IACSource`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.IDCSource`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.INCSource`
* The type object of :class:`hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator`
* An instance of :class:`hbp_nrp_cle.brainsim.BrainInterface.ICustomDevice`.

For both adapters, the instance of :class:`hbp_nrp_cle.brainsim.BrainInterface.ICustomDevice` connects to the neuronal simulator on its own, using the brain communication adapter. Thus, in this case the brain communication adapter must simply apply the custom device to itself.

For all of the other cases, there are two basic options.
1. The provided neurons parameter is a neuron population. In this case, a single device is created that has the given type.
2. The provided neurons parameter is a list of neuron populations. In this case, a device group is created where each device in this device group has the given type.

Further than neuron selector and device type, a device registration also contains additional parameters. These can be either directly forwarded to the device adapter or interpreted in any way.

The PyNN adapter
----------------

The PyNN adapter uses PyNN devices to fulfill the device interfaces. Additional configuration is forwarded to the PyNN devices where applicable, i.e. the adapter allows that TF device specifications contain configuration parameters that are not applicable for NEST.

Details to the PyNN adapter can be found in its code documentation: :class:`hbp_nrp_cle.brainsim.PyNNCommunicationAdapter.PyNNCommunicationAdapter`.

The Mock adapter
----------------

The mock communication adapter basically ignores the neuron specification since the devices are not connected to any neuronal simulation. Rather, the mock communication adapter allows to add additional device specifications called *updates* (only applicable for spike sinks). This *updates* property that is present on all devices allows to configure the devices to obtain new values at given simulation times.
These updates are specified as lists of tuples where the first item is the simulation time and the second item is the updated value. The only exception to this rule is the spike generator, here *updates* is only a list of simulation times where the spike detector should be marked as having detected spikes.

Conversely, spike source devices create a history of configuration values. These histories are also lists of tuples where the first item is the simulation time and the second one the value assigned to the device.

With these options to retain configuration or set configuration in advance, it is possible for neuroscience users to test TFs since the simulation can be run in a mocked environment where the spike sinks retain predefined values. The histories can then be queried for assertions.

The mock control adapter is a usual mock.