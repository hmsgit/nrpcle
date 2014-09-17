# Description 

The package contains the PyNN implementation of the Brain-to-Robot interface.
The PyNNCommunicationAdapter serves as communicator between the transfer functions
and the simulation back-end. The PyNNControlAdapter regulates the control of the
PyNNCommunicationAdapter. The folder "devices" contains implementations of
basic device types, which are connected to desired neurons during the simulation
and whose input/output is the output/input from/to the transfer functions.

# Device types

The devices which translate the sensory output of a robot to neuronal signals
are either spike generators or current generators:
PyNNPoissonSpikeGenerator: generates spikes whose distances are sampled from a
    Poisson distribution
PyNNDCSource: generates a direct current
PyNNACSource: generates an alternating current
PyNNNCSource: generates a noisy current

Devices, which produce neuronal signals to action signals of a robot are
either leaky integrate-and-fire neurons without threshold potential
(-> leaky integrators) or (temporary for reasons of testing) spike detectors
PyNNIFCurrAlpha: implementation of a leaky integrate-and-fire (IF) neuron with
current-based synapses and alpha-shaped (= difference of exponentials) post-
synaptic currents
PyNNSpikeDetector: detects whether one of the connected neurons has spiked
in the last time step and in that case returns a "1", otherwise "0"
