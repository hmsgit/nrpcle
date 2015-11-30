'''
Implementation of PyNNPoissonSpikeGenerator
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IPoissonSpikeGenerator
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = 'DimitriProbst'


class PyNNPoissonSpikeGenerator(AbstractBrainDevice, IPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    default_parameters = {
        "duration": float("inf"),
        "start": 0.0,
        "rate": 0.0,
        "connector": None,
        "weights": 0.00015,
        "delays": 0.1,
        "source": None,
        "target": "excitatory",
        "synapse_dynamics": None,
        "label": None,
        "rng": None
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Poisson spike generator.

        :param duration: Duration of spike train, default: infinity
        :param start: Start time of spike train, default: 0.0 ms
        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        :param connector: a PyNN Connector object, or, if neurons is
            a list of two populations, a list of two Connector objects
        :param source: string specifying which attribute of the presynaptic
            cell signals action potentials
        :param target: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_dynamics: a PyNN SynapseDy
        :param label: label of the Projection object
        :param rng: RNG object to be used by the Connector
            synaptic plasticity mechanisms to use
        """
        super(PyNNPoissonSpikeGenerator, self).__init__(**params)

        self.__generator = None

        self.create_device()

    @property
    def rate(self):
        """
        Returns the frequency of the Poisson spike generator
        """
        return self.__generator.get('rate')[0]

    @rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Poisson spike generator

        :param value: float
        """
        self.__generator.set('rate', value)

    def create_device(self):
        """
        Create Poisson spike generator device
        """
        self.__generator = sim.Population(1, sim.SpikeSourcePoisson,
                                          self.get_parameters("duration",
                                                              "start",
                                                              "rate"))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """

        if "connector" in self._parameters or not self._parameters["connector"]:
            if not (self._parameters["target"] == 'excitatory' or neurons.conductance_based):
                self._parameters["weights"] *= -1

            self._parameters["connector"] = \
                sim.AllToAllConnector(**self.get_parameters("weights",
                                                            "delays"))

        return sim.Projection(presynaptic_population=self.__generator,
                              postsynaptic_population=neurons,
                              **self.get_parameters("source",
                                                    "target",
                                                    ("method", "connector"),
                                                    "synapse_dynamics",
                                                    "label",
                                                    "rng"))
