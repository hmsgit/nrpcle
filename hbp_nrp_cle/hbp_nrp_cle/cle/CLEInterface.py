"""
This module represents the interfaces for the closed loop engine.
"""

__author__ = 'LorenzoVannucci'


# pylint: disable=R0921
class IClosedLoopControl(object):  # pragma: no cover
    """
    Represents the closed loop engine synchronization mechanism.
    """
    @property
    def network_file(self):
        """
        Gets or sets the neuronal network file

        :return: The path to the neuronal network
        """
        raise NotImplementedError("Method not implemented")

    @network_file.setter
    def network_file(self, value):
        """
        Gets or sets the neuronal network file
        """
        raise NotImplementedError("Method not implemented")

    @property
    def network_configuration(self):
        """
        Gets or sets the neuronal network configuration
        """
        raise NotImplementedError("Method not implemented")

    @network_configuration.setter
    def network_configuration(self, value):
        """
        Gets or sets the neuronal network configuration
        """
        raise NotImplementedError("Method not implemented")

    def initialize(self):  # -> None:
        """
        Initializes the closed loop engine.
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
