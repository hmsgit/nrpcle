__author__ = 'GeorgHinkel'


class ISpikeGenerator(object):  # pragma: no cover
    """
    Represents a communication object that may generate spikes
    """
    pass


class ICurrentGenerator(object):
    """
    Represents a communication object that may generate currents
    """
    pass


class IFixedFrequencySpikeGenerator(ISpikeGenerator):  # pragma: no cover
    """
    Represents a communication object that generates spikes on a fixed rate
    """

    def __get_rate(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_rate(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    rate = property(__get_rate, __set_rate)


class IPatternSpikeGenerator(ISpikeGenerator):  # pragma: no cover
    """
    Represents a spike generator generating spikes in a pattern
    """

    def __get_pattern(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_pattern(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    pattern = property(__get_pattern, __set_pattern)


class IDCSource(ICurrentGenerator):
    """
    Represents a current generator which generates direct current
    """

    def __get_amplitude(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_amplitude(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    amplitude = property(__get_amplitude, __set_amplitude)


class IACSource(ICurrentGenerator):
    """
    Represents a current generator which generates alternating current
    """

    def __get_amplitude(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_amplitude(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    amplitude = property(__get_amplitude, __set_amplitude)


class INCSource(ICurrentGenerator):
    """
    Represents a current generator which generates noisy current
    """

    def __get_mean(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_mean(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    mean = property(__get_mean, __set_mean)


class IPoissonSpikeGenerator(ISpikeGenerator):  # pragma: no cover
    """
    Represents a spike generator based on a Poisson Distribution
    """

    def __get_rate(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_rate(self, rate):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    rate = property(__get_rate, __set_rate)


class ISpikeDetector(object):  # pragma: no cover
    """
    Represents a communication object that may detect spikes
    """
    pass


class ISpikeRecorder(ISpikeDetector):  # pragma: no cover
    """
    Represents a communication object that records spikes from a certain neuron
    """

    def __get_recorded_spikes(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    recorded_spikes = property(__get_recorded_spikes)


class INeuronVoltmeter(ISpikeDetector):  # pragma: no cover
    """
    Represents a spike detector that integrates the spikes to the voltage of a neuron
    """

    def __get_voltage(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    voltage = property(__get_voltage)



class IIFCurrAlpha(ISpikeDetector):
    """
    Represents the membrane voltage of a current-based LIF neuron
    with alpha-shaped post synaptic currents. The neurons default threshold
    potential is set to infinity, so that the neuron never spikes, but
    only serves as a leaky integrator of the incoming spike train.
    """

    def __get_voltage(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    voltage = property(__get_voltage)

    def connect(self, neurons):
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class ICustomDevice(object):  # pragma: no cover
    """
    Represents device type with an internal logic that can be mapped to existing device types
    A brain communication adapter may chose whether to use the custom device or to support it directly
    """

    def apply(self, neurons, brain_adapter):
        """
        Apply the device type to the neurons with the given set of GIDs and the given adapter
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
        Requests a communication object with the given spike generator type for the given set of neurons
        :param neurons: A reference to the neurons to which the spike generator should be connected
        :param spike_generator_type: A spike generator type (see documentation for a list of allowed values)
        :param kwargs: A dictionary of configuration parameters
        :return: A communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def register_spike_sink(self, neurons, spike_detector_type, **kwargs):  # -> ISpikeDetector:
        """
        Requests a communication object with the given spike detector type for the given set of neurons
        :param neurons: A reference to the neurons which should be connected to the spike detector
        :param spike_detector_type: A spike detector type (see documentation for a list of allowed values)
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
