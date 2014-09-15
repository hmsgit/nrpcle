__author__ = 'GeorgHinkel'


class ISpikeGenerator(object):
    """
    Represents a communication object that may generate spikes
    """
    pass


class IFixedFrequencySpikeGenerator(ISpikeGenerator):
    """
    Represents a communication object that generates spikes on a fixed frequency
    """

    def __get_frequency(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_frequency(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    frequency = property(__get_frequency, __set_frequency)


class IPatternSpikeGenerator(ISpikeGenerator):
    """
    Represents a spike generator generating spikes in a pattern
    """

    def __get_pattern(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_pattern(self, value):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    pattern = property(__get_pattern, __set_pattern)


class IPoissonSpikeGenerator(ISpikeGenerator):
    """
    Represents a spike generator based on a Poisson Distribution
    """

    def __get_rate(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def __set_rate(self, rate):
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    rate = property(__get_rate, __set_rate)


class ISpikeDetector(object):
    """
    Represents a communication object that may detect spikes
    """
    pass


class ISpikeRecorder(ISpikeDetector):
    """
    Represents a communication object that records spikes from a certain neuron
    """

    def __get_recorded_spikes(self):  # -> list:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    recorded_spikes = property(__get_recorded_spikes)


class INeuronVoltmeter(ISpikeDetector):
    """
    Represents a spike detector that integrates the spikes to the voltage of a neuron
    """

    def __get_voltage(self):  # -> float:
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    voltage = property(__get_voltage)


class ICustomDevice(object):
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


class IBrainCommunicationAdapter(object):
    """
    Represents the communication interface to the neuronal simulator
    """

    def register_generate_spikes(self, neurons, spike_generator_type, **kwargs):  # -> ISpikeGenerator:
        """
        Requests a communication object with the given spike generator type for the given set of neurons
        :param kwargs: A dictionary of configuration parameters
        :param neurons: A reference to the neurons where the spike generator should be installed
        :param spike_generator_type: A spike generator type (see documentation for a list of allowed values)
        :return: A communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def register_consume_spikes(self, neurons, spike_detector_type, **kwargs):  # -> ISpikeDetector:
        """
        Requests a communication object with the given spike detector type for the given set of neurons
        :param neurons: A reference to the neurons where the spikes should be detected
        :param spike_detector_type: A spike detector type (see documentation for a list of allowed values)
        :param kwargs: A dictionary of configuration parameters
        :return: A Communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self):  # -> None:
        """
        Initializes the adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def refresh_buffers(self, t):  # -> None:
        """
        Refreshes all detector buffers
        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IBrainControlAdapter(object):
    """
    Represents a controller object for the neuronal simulator
    """

    def is_alive(self):  # -> bool:
        """
        Gets a status whether the neuronal simulator is still alive
        :return: True if the simulator is alive, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self):  # -> None:
        """
        Initializes the neuronal simulator
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