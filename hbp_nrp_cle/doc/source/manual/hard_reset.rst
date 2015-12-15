Changing Simulated Models during a running simulation
=====================================================

During a simulation, it is possible to change the models simulated in each of the simulators separately.
This may be done in order to continue to use the state of the other simulator or the internal state
of Transfer Functions. Besides actually changing the models, such a change may imply to reset the
communication adapters as well and with this also all Transfer Functions connected to adapter components
issued by the communication adapter as well.

- **Changing the Robot**
  For the default robot communication adapter, the :class:`hbp_nrp_cle.robotsim.RosCommunicationAdapter`,
  such a hard reset is not necessary as ROS topics are not closely coupled. In particular, it is transparent
  for a subscriber that a publisher has vanished and a new publisher is about to be created. As long as
  new publisher publishes messages on the same topic, the subscriber does not get noticed at all.

- **Changing the Neuronal Network**
  This situation is very different for the neuronal network simulation where the communication adapters
  may inject artificial neurons into the network. When the network is reset, these neurons cease to exist
  and must be reintroduced. This requires a functionality to discard the current devices connected to any
  of the transfer functions and replace them by reconnecting the transfer functions to neurons.

In the following, we thus concentrate on the implementation of changing the neuronal network during a
running simulation.

Implementation of Changing a Neuronal Network under the hood
------------------------------------------------------------

The CLE has direct support for changing the neuronal network underneath a running simulation during
the simulation. For this, the following steps are performed:

#. If the CLE is currently running, stop it.
#. If the brain adapter has already loaded a neuronal network, discard it.
#. Load the new neural network through the brain control adapter.
#. Perform a hard reset on the brain devices of the Transfer Functions.
   That is: Discard all devices created so far, then iterate through all the Transfer Functions and
   recreate all brain devices from scratch.

In particular, the following is not done:

- Reset the robot publishers and subscribers. In particular, if no new topic messages are sent, any
  topic subscriber will still have its old value.
- Reset the TF variables to their initial values. This means that any stateful Transfer Functions will
  not be reset.
- The position of the robot as well as all other models loaded in the robots environment.

To reuse one of these configurations may be a sufficient reason to not reset the entire simulation
after changing the neuronal network. This is in particular helpful, when only some parameters internal
to the neuronal network have been adjusted.

Necessary Considerations when changing a Neuronal Network
---------------------------------------------------------

The neuronal network represents the main control of the robot during a simulation and if this control
is exchanged, the simulation may fail. Thus, when changing a neuronal network during a simulation, the
following points ought to be taken into consideration:

- The simulation time is not reset. This means that the neuronal network now has a different clock than
  the robotic simulation. They are still being synchronized by the CLE in the sense that they operate on
  the same speed but may have different values.
- Recreating the Transfer Functions, the CLE will try to create new devices for all Transfer Functions
  currently present in the Transfer Function Manager. This will execute the neuron selectors on the
  changed neuronal network. If this selection fails as for example the network does no longer contains
  a population with this name, then the simulation fails.

Especially the last point means that as soon as one wants to change the topology of the network and
not just some connection or global variables, then these actions must be done very thoughtfully as
otherwise the simulation transitions to its failed state so that it cannot be resumed.

In particular, one must ensure that all neurons that were previously connected to a Transfer Function
still exist. This may be implemented by deleting all Transfer Functions that connect to neurons who
will not be there in the new network as only those Transfer Functions are reconnected that are in the
scope of the Transfer Function Manager when the network is changed.

Further, still all new Transfer Functions need to be valid at the time they are added to the CLE. In
particular, when a Transfer Function shall target neurons that will be introduced by a new network,
the neural network must be changed before the Transfer Function can be created.

Overall, this implies the following workflow:

#. Delete all Transfer Functions connecting to neurons that will cease to exist when the neural network
   is changed
#. Change the neural network
#. Add Transfer Functions connecting to neurons newly introduced in the neural network

Every other sequence will lead the simulation to a failed state.

.. note:: However, aside of the technical restrictions, one may also rethink that changing the neural
    network conflicts biological plausibility, usually an important goal when working with spiking
    neural networks. This is the reason the functionality is mainly focussed on simple parameter changes
    e.g. to compensate for missing implementation of learning when this is not used or to adjust
    parameters that cannot be learnt.

