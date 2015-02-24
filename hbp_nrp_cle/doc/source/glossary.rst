********
Glossary
********

This chapter describes some terminology that is used throughout this documentation.

====================== ===================================================
Term                   Explanation
====================== ===================================================
BIBI                   Brain and Body Integrator, the configuration of transfer functions through a simple and eventually graphical syntax. See :doc:`BIBI-configuration` for details.
Brain comm. adapter    The adapter that is used by the TF framework to connect with the neuronal simulation, e.g. to create suitable devices to connect with the parameters of a TF
CLC                    Abbreviation for Closed Loop Controller, see :doc:`CLE architecture<cle_architecture>`
CLE                    The Closed Loop Engine whose architecture can be found here: :doc:`CLE architecture<cle_architecture>`
Communication Object   Communication object is the generalization of neuronal network devices and robot publishers and subscribers. Thus, it represents objects that are accessed by the TF framework to connect parameters of a TF with a simulation in either way.
Device                 In neuronal simulation, devices are little programs that are injected into a neuronal network and run with the same clock as the neuronal simulation and can be accessed from outside. A typical example is a leaky integrator that basically returns the voltage of a neuron. The brain adapters of the CLE e.g. to PyNN do inject such devices into the neuronal network. However, within the TF framework, we also refer to the adapter objects that connect these devices with the TF framework devices, so we identify these adapters with the devices that they adapt. Devices may be either spike sinks or spike sources, i.e. either consume spikes of connected neurons or create spikes (or currents) and send them to connected neurons. Examples of spike sinks are leaky integrators that are essentially neurons that do not spike (infinite threshold voltage) but whose voltage is then accessed by the robot. Examples of spike sources are either current generators (AC, NC or DC source) or Poisson based spike generators.
Device Group           For brain simulators, it is often infeasible to work with single devices but whole groups. Consider for example an image recognition. If every pixel would be a spike generator device, the TF would need a number of parameters depending on the image resolution. A device group is a group of such devices that groups all these devices that logically belong together.
NEST                   NEST is the neuronal simulator that we currently use by default, see http://www.nest-initiative.org/
PyNN                   An interface for neuronal simulators, see http://neuralensemble.org/PyNN/
Robot Publisher        A robot publisher is the equivalent of a spike source device on the robot side, but only for sending data to the robot. As we are currently using ROS, robot publishers are really ROS publishers sending data to some Gazebo topics.
Robot Subscriber       A robot subscriber is the equivalent of a spike sink device, i.e. it is a port for the incoming data.
Robot comm. adapter    The adapter that is used by the TF framework to connect with the robot simulation, e.g. to create suitable robot subscribers and robot publishers in accordance with the used input.
Transfer Function (TF) A function that interconnects the neuronal simulator with a (currently simulated) robot. This includes the function itself as well as annotation how to connect its parameters to the neuronal simulation or to the robot simulation. Thus, TFs are end to end and cannot be stacked together. However, their functional specification (the body) can be stacked.
TF node/TF manager     An organizational unit for the TFs. The terms TF node and TF manager are used interchangeably. Each TF must be connected to exactly one TF manager that manages its execution. By default, this is the currently active instance.
WSE                    World Simulation Engine, the generalization of the robot simulation. We currently use Gazebo (see http://gazebosim.org/) through a ROS (see http://www.ros.org/) interface as our World Simulation Engine.
====================== ===================================================
