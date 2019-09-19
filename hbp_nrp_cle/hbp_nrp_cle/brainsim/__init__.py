"""
Description
-----------

The package contains the Brain adapter implementations of the Brain-to-Robot interface.
The CommunicationAdapter serves as communicator between the transfer functions
and the simulation back-end. The ControlAdapter regulates the control of the
CommunicationAdapter. The folder "__devices" contains implementations of
basic device types, which are connected to desired neurons during the simulation
and whose input/output is the output/input from/to the transfer functions.

Device types
------------

The devices which translate the sensory output of a robot to neuronal signals
are either spike generators or current generators:
* IPoissonSpikeGenerator: generates spikes whose distances are sampled from a
Poisson distribution
* IFixedSpikeGenerator: generates equidistant spikes at a given frequency
* IDCSource: generates a direct current
* IACSource: generates an alternating current
* INCSource: generates a noisy current

Devices, which produce neuronal signals to action signals of a robot are
either leaky integrate-and-fire neurons without threshold potential
(-> leaky integrators) or (temporary for reasons of testing) spike detectors

* ILeakyIntegratorExp: implementation of a leaky integrate-and-fire (IF) neuron with
                 current-based synapses and decaying-exponential post-synaptic currents
* ILeakyIntegratorAlpha: implementation of a leaky integrate-and-fire (IF) neuron with
                   current-based synapses and alpha-shaped (= difference of
                   exponentials) post-synaptic currents
* ISpikeRecorder: detects whether one of the connected neurons has spiked in the last time
                     step and in that case returns a "1", otherwise "0"
* IPopulationRate: measured the firing rate of the connected neuronal population

The devices can be configured in the same way as NEST devices, any additional configuration
parameters are directly forwarded to the created NEST devices.
"""

__author__ = 'Georg Hinkel'

# We need to import nest here, otherwise we get a segfaults in tests
import pyNN.nest

from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter, IBrainControlAdapter

simulator = None

from mpi4py import MPI
# Duplicate of COMM_WORLD mpi communicator to be used within NRP so communications don't interfere
# with NEST
COMM_NRP = MPI.COMM_WORLD.Dup()
