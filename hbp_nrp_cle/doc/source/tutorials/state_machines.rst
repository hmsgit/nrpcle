Tutorial: Writing state machines for experiments
================================================

The NRP is designed to use state machines to control and evaluate experiments. A state machine controlling an experiment can monitor any simulation properties published on ROS topics (e.g. simulation time, sensor output, spiking activity of brain) and publish on ROS topics or call ROS services. A state machine evaluating the success of an experiment has the same capabilities for monitoring a simulation and should either publish a success or failure message to the NRP ROS status topic.

The NRP uses SMACH for executing state machines, hence state machines have to be specified in Python.


Controlling an experiment
-------------------------

The following examples show how to specify state machines for certain tasks.

Trigger an action after 30 seconds of simulation time
"""""""""""""""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/time_event.py
   :language: python

This example contains two states: a `MonitorState <http://wiki.ros.org/smach/Tutorials/MonitorState>`_ monitoring the '/ros_cle_simulation/status' ROS topic as condition and a `ServiceSate <http://wiki.ros.org/smach/Tutorials/ServiceState>`_ calling the '/gazebo/set_visual_properties' ROS service to change the screen color as action. The MonitorState executes a callback each time a message is received on the topic.


Trigger an action based on user interaction
"""""""""""""""""""""""""""""""""""""""""""

.. note::
    Currently, the ExD back-end directly executes ROS services upon receiving user interactions/events. In the future the name of the user interaction should be published on a ROS topic to allow the control state machine to execute the user interaction/event.


Trigger an action based on a monitored property
"""""""""""""""""""""""""""""""""""""""""""""""

The MonitorState allows to monitor any available ROS topic. Thus, monitoring a property can be done in the same way as for simulation time.

.. literalinclude:: sm_examples/spike_event.py
   :language: python

In this example a neuron monitor, set up in the according BIBI configuration file, is used to monitor brain activity and use it as condition for an event.


Combined events
---------------

The following examples show how to combine several events into a single state machine.

Multiple time events as sequence
""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_time_events.py
   :language: python

In this example the first event is triggered after 30 seconds and the second one after 60 seconds. The actions of both events are implemented by a `CBState <http://wiki.ros.org/smach/Tutorials/CBState>`_, simply executing a callback.


Multiple conditions for a single event
""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_conditions_event.py
   :language: python

This example shows how to combine multiple conditions for a single event by using a `Concurrence Conatiner <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`_. Both conditions, simulation time larger than 30 seconds and brain activity above threshold, have to be met. The order the conditions become true doesn't matter.


Multiple independent events in a single state machine
"""""""""""""""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_events_sm.py
   :language: python

This example shows how to combine multiple events in a single state machine by nesting `State Machine Containers <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`_ (events) into a `Concurrence Conatiner <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`_. The concurrence container terminates a soon as both nested state machines (events) terminate.


Evaluating experiment result
----------------------------

A state machine for evaluating the experiment success does not differ from an experiment control state machine in terms of capabilities. However, it should have exactly two final states: one publishing a 'success' message on the CLE status topic and one publishing a 'failure' message.

.. note::
    Currently, the front-end is not able to visualize success or failure messages.

