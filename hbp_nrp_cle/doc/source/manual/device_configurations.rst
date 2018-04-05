Configuration parameters of devices
===================================

In this section, we list the parameters of all device types

AC Current Source
-----------------

#. **amplitude:** The amplitude of the current, default: 1.0
#. **offset:** The offset of the current, default: 0.0
#. **frequency:** The frequency of the alternation, default: 10.0
#. **phase:** The phase to begin with, default: 0.0
#. **start:** The simulation time when the current injection starts, default: 0.0
#. **stop:** The simulation time when the current injection ends, default: infinity

DC Current Source
-----------------

#. **amplitude:** The amplitude of the current, default: 1.0
#. **start:** The simulation time when the current injection starts, default: 0.0
#. **stop:** The simulation time when the current injection ends, default: infinity
#. **parrot:** Indicates whether a separate current source should be generated even when the neuron model has direct support for DC current generators, default false

Fixed Spike Generator
---------------------

#. **initial_rate:** The initial rate of the spike generator, default: 0.0
#. **connector:** The connector for the connected synapse, default: connect all provided neurons
#. **weight:** The weight of the connection synapse, default: uniform random distribution [0, 0.01]
#. **delay:** The delay of the connection synapse, default: None
#. **receptor_type:** The receptor type of the connection synapse, default: 'excitatory'
#. **synapse_type:** The synapse type of the connection synapse, default: static synapse

Leaky Integrator Alpha
----------------------

#. **v_rest:**  default: 0.0
#. **connector:**  default: connect all provided neurons
#. **weight:**  default: 0.01
#. **delay:**  default: 0.1
#. **receptor_type:**  default: 'excitatory'
#. **synapse_type:**  default: static synapse

Leaky Integrator Exp
--------------------

#. **v_rest:**  default: 0.0
#. **connector:**  default: connect all provided neurons
#. **weight:**  default: uniform random distribution [0, 0.01]
#. **delay:**  default: uniform random distribution [0.1, 2.0]
#. **receptor_type:**  default: 'excitatory'
#. **synapse_type:**  default: static synapse

Noisy Current Source
--------------------

#. **mean:** The mean of the generated noisy current, default: 0.0
#. **stdev:** The standard deviation of the injected current, default: 1.0
#. **start:** The simulation time when the current injection starts, default: 0.0
#. **stop:** The simulation time when the current injection ends, default: infinity

Poisson-Generator
-----------------

#. **duration:** How long spikes should be generated default: infinity
#. **start:** When the spike generator should start generating spikes, default: 0.0
#. **rate:** The rate at which spikes should be generated, default: 0.0
#. **connector:** The connector for the connection synapse, default: connect all provided neurons
#. **weight:** The weight of the connection synapse, default: 0.00015
#. **delay:** The delay of the connection synapse, default: 0.1
#. **receptor_type:** The receptor type of the connection synapse, default: "excitatory"
#. **synapse_type:** The type of the connector synapse, default: static synapse
#. **n:** The number of Poisson generators that should be created, default: 1

Population-Rate
---------------

A population rate has no parameters.

Spike Recorder
--------------

#. **use_ids:** Whether the spike recorder should use the global GIDs of the neurons (otherwise indices within the populations are used), default true
