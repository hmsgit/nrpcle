"""
This module represents the interfaces for the closed loop engine.
"""

__author__ = 'LorenzoVannucci'


# pylint: disable=R0921
class IClosedLoopControl(object):  # pragma: no cover
    """
    Represents the closed loop engine synchronization mechanism.
    """
    def load_network_from_file(self, network_file, network_configuration):
        """
        Load (or reload) the brain model from a file the neuronal network file

        :param network_file: A python PyNN script or an h5 file
        containing the neural network definition
        :param network_configuration: A dictionary indexed by population names and
        containing neuron indices. Neuron indices could be defined a single integer,
        list of integers or python slices. Python slices could be replaced by a
        dictionary containing the 'from', 'to' and 'step' values.
        """
        raise NotImplementedError("Method not implemented")

    def initialize(self, network_file, configuration):  # -> None:
        """
        Initializes the closed loop engine.
        :param network_file: A python PyNN script or an h5 file
         containing the neural network definition
        :param configuration: A set of populations
        :return:
        """
        raise NotImplementedError("Method not implemented")

    @property
    def is_initialized(self):  # -> bool
        """
        Returns True if the simulation is initialized, False otherwise.
        """
        raise NotImplementedError("Method not implemented")

    def run_step(self, timestep):  # -> float64:
        """
        Runs both simulations for the given time step in seconds.

        :param timestep: The CLE time step in seconds
        :return: Updated simulation time, otherwise -1
        """
        raise NotImplementedError("Method not implemented")

    def shutdown(self):  # -> None:
        """
        Shuts down both simulations.
        """
        raise NotImplementedError("Method not implemented")

    def start(self):  # -> None:
        """
        Starts the orchestrated simulations.
        """
        raise NotImplementedError("Method not implemented")

    def stop(self):  # -> None:
        """
        Stops the orchestrated simulations. Also waits for the current
        simulation step to end.
        """
        raise NotImplementedError("Method not implemented")

    def reset(self):  # -> None:
        """
        Reset the orchestrated simulations.
        """
        raise NotImplementedError("Method not implemented")

    def reset_world(self, sdf_world_string):
        """
        Reset the world to the configuration described by sdf_world_string.
        """
        raise NotImplementedError("Method not implemented")

    @property
    def simulation_time(self):  # -> float64
        """
        Get the current simulation time.
        """
        raise NotImplementedError("Method not implemented")

    @property
    def real_time(self):  # -> float64
        """
        Get the total execution time.
        """
        raise NotImplementedError("Method not implemented")

    def tf_elapsed_time(self):
        """
        Gets the time share of the Transfer Functions
        """
        raise NotImplementedError("Method not implemented")

    def brainsim_elapsed_time(self):
        """
        Gets the time share of the brain simulation
        """
        raise NotImplementedError("Method not implemented")

    def robotsim_elapsed_time(self):
        """
        Gets the time share of the robot simulation
        """
        raise NotImplementedError("Method not implemented")

    def wait_step(self):  # -> None
        """
        Wait for the currently running simulation step to end.
        """
        raise NotImplementedError("Method not implemented")
