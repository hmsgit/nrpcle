'''
Implementation of PyNNNCSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import INCSource
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = 'DimitriProbst'


class PyNNNCSource(AbstractBrainDevice, INCSource):
    """
    Represents a noisy current generator
    :param kwargs: Optional configuration parameters
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a noisy current generator.

        :param mean: Mean value of the noisy current, default: 0.0 nA
        :param stdev: Standard deviation of the noisy current, default: 1.0 nA
        :param dt: Interval between updates of the current amplitude in ms,
            default: simulation time step
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        :param rng: an RNG object from the `pyNN.random` module,
            default: NativeRNG of the back-end
        """
        self._generator = None

        self.create_device(params)

    @property
    def mean(self):
        """
        Returns the mean of the current
        """
        return self._generator.mean

    # pylint: disable=unused-argument
    # pylint: disable=no-self-use
    @mean.setter
    def mean(self, value):
        """
        Sets the mean of the current

        :param value: float
        """

        raise RuntimeError("Resetting this property is currently not supported by PyNN")

    @property
    def stdev(self):
        """
        Returns the stdev of the current
        """
        return self._generator.stdev

    # pylint: disable=unused-argument
    # pylint: disable=no-self-use
    @stdev.setter
    def stdev(self, value):
        """
        Sets the stdev of the current

        :param value: float
        """

        raise RuntimeError("Resetting this property is currently not supported by PyNN")

    def create_device(self, params):
        """
        Create a noisy current source

        :param params: Dictionary of neuron configuration parameters
        :param mean: Mean value of the noisy current, default: 0.0 nA
        :param stdev: Standard deviation of the noisy current, default: 1.0 nA
        :param dt: Interval between updates of the current amplitude in ms,
            default: simulation time step
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        :param rng: an RNG object from the `pyNN.random` module,
            default: NativeRNG of the back-end
        """
        self._generator = sim.NoisyCurrentSource(
            mean=params.get('mean', 0.0),
            stdev=params.get('stdev', 1.0),
            dt=params.get('dt', None),
            start=params.get('start', 0.0),
            stop=params.get('stop', None),
            rng=params.get('rng', sim.NativeRNG()))

    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
        Assembly object
        """
        self._generator.inject_into(neurons)
