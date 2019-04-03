# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module represents the interfaces for the brain communication and control adapter
"""

__author__ = 'GeorgHinkel, Sebastian Krach'


class PopulationInfo(object):  # pragma: no cover
    """
    Gathers information about neuron populations
    """

    @property
    def name(self):
        """
        Gets the population name
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def celltype(self):
        """
        Gets the celltype of the population
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def parameters(self):
        """
        Gets the parameters of a the population as dict
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def gids(self):
        """
        Gets the global unique identifiers for this population
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def indices(self):
        """
        Gets the population indices
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IBrainDevice(object):  # pragma: no cover
    """
    Device to connect to brain simulation
    """

    def connect(self, neurons):
        """
        Connects the brain device to the specified neuron population.
        :param neurons: the neurons of the brain to which the device will connect
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self, transfer_function_manager):
        """
        Resets the device

        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def _disconnect(self):
        """
        INTERNAL USE ONLY: this should never be directly invoked by a user.

        Disconnects the brain device from any output neuron populations. This device will no longer
        interact with the brain after disconnect is called.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def active(self):
        """
        Returns the activation state of this device
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @active.setter
    def active(self, bool_value):
        """
        Sets the activation state of this device
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IDeviceGroup(IBrainDevice):  # pragma: no cover
    """
    Gathers multiple devices to a group
    """

    def connect(self, neurons, **params):
        """
        Connects the devices contained in this device group to the specified neuron
        population. If the neuron population has the same size as the amount of devices contained
        in the device group a One-to-One connection is used.

        :param neurons: the neurons of the brain to which the device will connect
        :param params: additional, device specific parameters. For each parameter either the
            value can be supplied, or a list of values, one for each nested device.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IFixedSpikeGenerator(IBrainDevice):  # pragma: no cover
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


class IPoissonSpikeGenerator(IBrainDevice):  # pragma: no cover
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


class ISpikeInjector(IBrainDevice):  # pragma: no cover
    """
    Represents a spike injector that is able to inject single spikes
    """

    def inject_spikes(self):
        """
        Injects a spike to the connected population
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IDCSource(IBrainDevice):  # pragma: no cover
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


class IACSource(IBrainDevice):  # pragma: no cover
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


class INCSource(IBrainDevice):  # pragma: no cover
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


class ISpikeRecorder(IBrainDevice):  # pragma: no cover
    """
    Represents a device that captures whether a neuron has spiked since the last iteration
    """

    @property
    def spiked(self):  # -> bool
        """
        Gets a value indicating whether the neuron has spiked since the last iteration
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def times(self):
        """
        Returns the times and neuron IDs of the recorded spikes within the last time step.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class ILeakyIntegratorAlpha(IBrainDevice):  # pragma: no cover
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


class ILeakyIntegratorExp(IBrainDevice):  # pragma: no cover
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


class IPopulationRate(IBrainDevice):  # pragma: no cover
    """
    Represents a device which returns the spiking frequency of a population of neurons
    """

    @property
    def rate(self):  # -> float:
        """
        Gets the current rate of the neuron population
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRawSignal(IBrainDevice):  # pragma no cover
    """
    Represents a device that allows for the exchange of raw data values. There is no direct neural
    equivalent as the semantics of the device depend on the semantics of the neural simulator.
    """

    @property
    def value(self):
        """
        Gets the current value.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @value.setter
    def value(self, new_value):
        """
        Sets the value to new_value
        :param new_value: the new value
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

    def reset(self, transfer_function_manager):
        """
        Resets the device

        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
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
        :param spike_generator_type: A spike generator type (:doc:`brain_adapter`)
        :param kwargs: A dictionary of configuration parameters
        :return: A communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def unregister_spike_source(self, device):
        """
        Disconnects and unregisters the given spike generator device.

        :param device: The spike generator device to deregister.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def activate_device(self, device, activate):
        """
        Change the activation status of the device.
        :param device: the device on which to apply the change
        :param activate: a boolean value denoting the new activation state
        :raises AttributeError the device doesn't support activation
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def register_spike_sink(self, neurons, spike_detector_type, **kwargs):  # -> ISpikeDetector:
        """
        Requests a communication object with the given spike detector type for the given set of
        neurons

        :param neurons: A reference to the neurons which should be connected to the spike detector
        :param spike_detector_type: A spike detector type (see :doc:`brain_adapter`)
        :param kwargs: A dictionary of configuration parameters
        :return: A Communication object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def unregister_spike_sink(self, device):
        """
        Disconnects and unregisters the given spike detector device.

        :param device: The spike detector device to deregister.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def refresh_buffers(self, t):  # -> None:
        """
        Refreshes all detector buffers

        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def shutdown(self):
        """
        Shuts down the brain communication adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def is_population(self, population):  # -> Boolean:
        """
        Determines whether the given object is a population

        :param population: The object that may be a population
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def create_view(self, population, sl):  # -> Object:
        """
        Creates a view of the given population

        :param population: The base population
        :param sl: The slice of the population that represents the view
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IBrainControlAdapter(object):  # pragma: no cover
    """
    Represents a controller object for the neuronal simulator
    """
    def load_populations(self, **populations):
        """
        Load (or reload) populations defined on the currently loaded brain.
        CAUTION: Populations become invalid when the brain they are defined on is changed.

        :param populations: A dictionary indexed by population names and
          containing neuron indices. Neuron indices can be defined by a single integer,
          list of integers or python slices. Python slices can be replaced by a
          dictionary containing the 'from', 'to' and 'step' values.
        :raise Exception: When no brain is currently loaded
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def load_brain(self, brain_file, **populations):
        """
        Loads the neuronal network contained in the given file

        :param brain_file: The path to the neuronal network file
        :param populations: Additional populations to create (Optional)
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def get_populations(self):
        """
        Gets an information about the populations currently available

        :return: A list of population infos
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

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

    def reset(self):  # -> None:
        """
        Resets the neuronal simulator
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
