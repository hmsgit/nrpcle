'''
Implementation of PyNNIFCurrAlpha
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IIFCurrAlpha
import warnings
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNIFCurrAlpha(IIFCurrAlpha):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    def __init__(self, params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to "infinity"
        is set to infinity by default in order to forbid the neuron to elicit
        spikes.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__cell = None
        self.latest_voltage = params.get('v_rest', -65.0)

        self.create_device(params)
        self.start_record_voltage()

    def __get_voltage(self):
        return self.__cell.get_v()[-1, -1]

    voltage = property(__get_voltage)

    def create_device(self, params):
        '''
        Creates a LIF neuron with alpha-shaped post synaptic currents
        and current-based synapses
        :param params: Dictionary of neuron configuration parameters
        '''
        cellparams = {'v_thresh': params.get('v_thresh', float('inf')),
                      'cm': params.get('cm', 1.0),
                      'tau_m': params.get('tau_m', 20.0),
                      'tau_syn_E': params.get('tau_syn_E', 0.5),
                      'tau_syn_I': params.get('tau_syn_I', 0.5),
                      'v_rest': params.get('v_rest', -65.0),
                      'v_reset': params.get('v_reset', -65.0),
                      'tau_refrac': params.get('tau_refrac', 0.1),
                      'i_offset': params.get('i_offset', 0.0)}
        self.__cell = sim.Population(1, sim.IF_curr_alpha, cellparams)

    def start_record_voltage(self):
        '''
        Records the voltage of the neuron
        '''
        self.__cell.record_v()

    def connect(self, neurons, **kwargs):
        """
        Connects the neurons specified in the list "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        connector = kwargs.get('connector', None)
        source = kwargs.get('source', None)
        target = kwargs.get('target', 'excitatory')
        synapse_dynamics = kwargs.get('synapse_dynamics', None)
        label = kwargs.get('label', None)
        rng = kwargs.get('rng', None)

        if connector is None:
            warnings.warn("Default weights and delays are used.", UserWarning)
            weights = sim.RandomDistribution('uniform', [0.0, 0.01])
            delays = sim.RandomDistribution('uniform', [0.1, 2.0])
            connector = sim.AllToAllConnector(weights=weights, delays=delays)
        return sim.Projection(presynaptic_population=neurons,
                              postsynaptic_population=self.__cell,
                              method=connector, source=source, target=target,
                              synapse_dynamics=synapse_dynamics, label=label,
                              rng=rng)

    def refresh(self):
        '''
        Refreshes the voltage value
        '''
        self.latest_voltage = self.__get_voltage()
        return self.latest_voltage
