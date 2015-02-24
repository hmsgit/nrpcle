BIBI Configuration File Format
==============================

Overview
--------

:num:`Fig. #bibi-cd` shows a quick overview on the BIBI configuration file format.

.. _bibi-cd:
.. figure:: img/bibi_class_diagram.png
   :width: 100%

   BIBI Configuration metamodel

The root element of a BIBI configuration is a BIBIConfiguration element. This element specifies the used robot model (bodyModel) as an SDF file (means a file path with ending .sdf). This is accompanied by the definition of the brain model. This definition consists of the path to the HDF5 file that contains the brain model as well as a range of neuron groups which can be accessed with the transfer functions. The neuron groups each define a population subset with some indices. We support three methods of selecting neurons from the whole circuit, by index, by a range and by a list for specifying multiple neurons.

Flow Expressions
----------------

Flow expressions are the primary concept for the BIBI configuration to express variability. In its simplest form, the flow expression can be a constant value. To react on simulation results, we support argument references. Such a reference can delegate to a local variable or one of the parameters of the transfer function, i.e. a topic channel or device(group). However, flow expressions also allow to scale arguments, access their properties or apply basic arithmetic operations such as add, subtract, multiply, divide, min or max. Furthermore, calls to externally implemented functionality are also allowed. In here, these calls may be stacked.

Transfer Functions
------------------

Transfer Functions can either be Robot2Neuron or Neuron2Robot, indicating their main direction as according to Transfer Functions Architecture. Transfer Functions have four different assets: Locals, TopicChannels, DeviceChannels and DeviceGroupChannels. Each of these assets can be read from or written to. This choice is made by specifying a body (than the result of this body is written to the asset) or leaving out a body. The exception to this rule are locals that always have a body and are both read from and written to.

Locals
------

Locals are local variables of the resulting transfer function. Their primary purpose is for later reference. Locals can be used in any TF. Locals should always be written to as the corresponding local variable would otherwise be unassigned.

Topic Channels
--------------

A topic chanel is the specification of a used topic that the TF will either read from or write to. It must specify the address (string with regular expression (/[\w_]+)+ where \w and the type of the robot topic. If no body is given, the topic channel is subscribed to, otherwise the body is published.

Device Channels
---------------

Likewise, a device channel is the specification of a device usage. It consists of a neuron selector and a device type. The possible device types are depicted in :num:`Fig. #bibi-device-types`.

.. _bibi-device-types:
.. figure:: img/bibi_deviceTypes.png
   :align: center

   Device types in the BIBI Configuration file format

The explanations for these device types can be found in the :doc:`Transfer Functions Architecture<transfer_functions>`. Note that some device types imply a body whereas others prohibit it. Any mistake here may result in an exception when loading the BIBI configuration.

Next to the device type, the device channels also need a specification of the neurons that they should be connected to.

Device Group Channels
---------------------

Device group channels are like device channels except that they target a group of devices instead of a single device.
