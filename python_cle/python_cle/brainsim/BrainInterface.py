"""
This module represents the interfaces for the brain communication and control adapter
"""

__author__ = 'GeorgHinkel'


class IDeviceGroup(object):
    """
    Gathers multiple devices to a group
    """
    pass


class ISpikeGenerator(object):  # pragma: no cover
    """
    Represents a communication object that may generate spikes
    """
    pass


class ICurrentGenerator(object):  # pragma: no cover
    """
    Represents a communication object that may generate currents
    """
    pass


class IFixedSpikeGenerator(ISpikeGenerator):  # pragma: no cover
    """
    Represents a communication object that generates spikes on a fixed rate
    """

    @property
    def rate(self):  # -> float:
        """
        Gets or sets the rate in which spikes should be generated
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @rate.setter
    def rate(self, value):
        """
        Sets the rate in which spikes should be generated
        :param value: The new rate in which spikes are generated
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IPoissonSpikeGenerator(ISpikeGenerator):  # pragma: no cover
    """
    Represents a spike generator based on a Poisson Distribution
    """

    @property
    def rate(self):  # -> float:
        """
        Gets or sets the rate in which spikes should be generated
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @rate.setter
    def rate(self, value):
        """
        Sets the rate in which spikes should be generated
        :param value: The new rate in which spikes are generated
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IDCSource(ICurrentGenerator):  # pragma: no cover
    """
    Represents a current generator which generates direct current
    """

    @property
    def amplitude(self):  # -> list:
        """
        Gets or sets the amplitude of this current generator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude to the new value
        :param value: The new amplitude
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IACSource(ICurrentGenerator):  # pragma: no cover
    """
    Represents a current generator which generates alternating current
    """

    @property
    def amplitude(self):  # -> list:
        """
        Gets or sets the amplitude of this current generator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude to the new value
        :param value: The new amplitude
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class INCSource(ICurrentGenerator):  # pragma: no cover
    """
    Represents a current generator which generates noisy current
    """

    @property
    def mean(self):  # -> list:
        """
        Gets or sets the mean value for the noisy current
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @mean.setter
    def mean(self, value):
        """
        Sets the mean value for the noisy current to the given value
        :param value: The new mean current
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class ISpikeDetector(object):  # pragma: no cover
    """
    Represents a communication object that may detect spikes
    """
    pass


class ISpikeRecorder(ISpikeDetector):  # pragma: no cover
    """
    Represents a device that captures whether a neuron has spiked since the last iteration
    """

    @property
    def spiked(self):  # -> bool
        """
        Gets a value indicating whether the neuron has spiked since the last iteration
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class ILeakyIntegratorAlpha(ISpikeDetector):  # pragma: no cover
    """
    Represents the membrane voltage of a current-based LIF neuron
    with alpha-shaped post-synaptic currents. The neurons default threshold
    potential is set to infinity, so that the neuron never spikes, but
    only serves as a leaky integrator of the incoming spike train.
    """

    @property
    def voltage(self):  # -> float:
        """
        Gets the current voltage of the voltmeter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class ILeakyIntegratorExp(ISpikeDetector):  # pragma: no cover
    """
    Represents the membrane voltage of a current-based LIF neuron
    with decaying-exponential post-synaptic currents. The neurons default
    threshold potential is set to infinity, so that the neuron never spikes,
    but only serves as a leaky integrator of the incoming spike train.
    """

    @property
    def voltage(self):  # -> float:
        """
        Gets the current voltage of the voltmeter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IPopulationRate(ISpikeDetector):  # pragma: no cover
    """
    Represents a device which returns the spiking frequency of a population of neurons
    """

    @property
    def rate(self):  # -> float:
        """
        Gets the current rate of the neuron population
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class ICustomDevice(object):  # pragma: no cover
    """
    Represents device type with an internal logic that can be mapped to existing device types
    A brain communication adapter may chose whether to use the custom device or to support it
    directly
    """

    def apply(self, neurons, brain_adapter, **config):
        """
        Apply the device type to the neurons with the given set of GIDs and the given adapter
        :param config: Additional device configuration
        :param neurons: A list of neuron GIDs for which to create the custom device
        :param brain_adapter: The brain communication adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IBrainCommunicationAdapter(object):  # pragma: no cover
    """
    Represents the communication interface to the neuronal simulator
    """

    def initialize(self):  # -> None:
        """
        Initializes the adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def register_spike_source(self, neurons, spike_generator_type, **kwargs):  # -> ISpikeGenerator:
        """
        Requests a communication object with the given spike generator type for the given set of
        neurons
        :param neurons: A reference to the neurons to which the spike generator should be connected
        :param spike_generator_type: A spike generator type (see documentation for a list of allowed
         values)
        :param kwargs: A dictionary of configuration parameters
        :return: A communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def register_spike_sink(self, neurons, spike_detector_type, **kwargs):  # -> ISpikeDetector:
        """
        Requests a communication object with the given spike detector type for the given set of
        neurons
        :param neurons: A reference to the neurons which should be connected to the spike detector
        :param spike_detector_type: A spike detector type (see documentation for a list of allowed
        values)
        :param kwargs: A dictionary of configuration parameters
        :return: A Communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def refresh_buffers(self, t):  # -> None:
        """
        Refreshes all detector buffers
        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IBrainControlAdapter(object):  # pragma: no cover
    """
    Represents a controller object for the neuronal simulator
    """

    def initialize(self):
        """
        Initializes the neuronal simulator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def is_alive(self):  # -> bool:
        """
        Gets a status whether the neuronal simulator is still alive
        :return: True if the simulator is alive, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def run_step(self, dt):  # -> None:
        """
        Runs the neuronal simulator for the given amount of simulated time
        :param dt the simulated time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
