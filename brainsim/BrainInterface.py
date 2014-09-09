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

    def __get_frequency(self):
        raise Exception("Not Implemented")

    def __set_frequency(self, value):
        raise Exception("Not Implemented")

    frequency = property(__get_frequency, __set_frequency)


class IPatternSpikeGenerator(ISpikeGenerator):
    """
    Represents a spike generator generating spikes in a pattern
    """

    def __get_pattern(self):
        raise Exception("Not Implemented")

    def __set_pattern(self, value):
        raise Exception("Not Implemented")

    pattern = property(__get_pattern, __set_pattern)


class IPoissonSpikeGenerator(ISpikeGenerator):
    """
    Represents a spike generator based on a Poisson Distribution
    """

    def __get_rate(self):  # -> int:
        raise Exception("Not Implemented")

    def __set_rate(self, rate):  # -> int:
        raise Exception("Not Implemented")

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
        raise Exception("Not Implemented")

    recorded_spikes = property(__get_recorded_spikes)


class INeuronVoltmeter(ISpikeDetector):
    """
    Represents a spike detector that integrates the spikes to the voltage of a neuron
    """

    def __get_voltage(self):  # -> float:
        raise Exception("Not Implemented")

    voltage = property(__get_voltage)


class IBrainCommunicationAdapter(object):
    """
    Represents the communication interface to the neuronal simulator
    """

    def register_generate_spikes(self, neurons, spike_generator_type):  # -> ISpikeGenerator:
        """
        Requests a communication object with the given spike generator type for the given set of neurons
        :param neurons: A reference to the neurons where the spike generator should be installed
        :param spike_generator_type: A spike generator type (see documentation for a list of allowed values)
        :return: A communication object
        """
        if not neurons in self.__generated_neurons:
            subscriber = self.create_spike_generator(neurons, spike_generator_type)
            self.__generated_neurons[neurons] = subscriber
            return subscriber
        else:
            return self.__generated_neurons[neurons]

    def register_consume_spikes(self, neurons, spike_detector_type):  # -> ISpikeDetector:
        """
        Requests a communication object with the given spike detector type for the given set of neurons
        :param neurons: A reference to the neurons where the spikes should be detected
        :param spike_detector_type: A spike detector type (see documentation for a list of allowed values)
        :return: A Communication object
        """
        if not neurons in self.__received_neurons:
            publisher = self.create_spike_detector(neurons, spike_detector_type)
            self.__received_neurons[neurons] = publisher
            return publisher
        else:
            return self.__received_neurons[neurons]

    def initialize(self):  # -> None:
        """
        Initializes the adapter
        """
        raise Exception("Not Implemented")

    def refresh_buffers(self, t):  # -> None:
        """
        Refreshes all detector buffers
        :param t: The simulation time
        """
        raise Exception("Not Implemented")


class IBrainControlAdapter(object):
    """
    Represents a controller object for the neuronal simulator
    """

    def is_alive(self):  # -> bool:
        """
        Gets a status whether the neuronal simulator is still alive
        :return: True if the simulator is alive, otherwise False
        """
        raise Exception("Not Implemented")

    def initialize(self):  # -> None:
        """
        Initializes the neuronal simulator
        """
        raise Exception("Not Implemented")

    def run_step(self, dt):  # -> None:
        """
        Runs the neuronal simulator for the given amount of simulated time
        :param dt the simulated time
        """
        raise Exception("Not Implemented")

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        raise Exception("Not Implemented")