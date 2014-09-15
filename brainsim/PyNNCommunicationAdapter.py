'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IBrainCommunicationAdapter, \
    IIFCurrAlpha, IPoissonSpikeGenerator, IDCSource, IACSource, INCSource
import pyNN.nest as sim
import warnings
import numpy as np

__author__ = 'DimitriProbst'


class PyNNCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """

    def __init__(self):
        """
        Initializes the communication adapter
        """
        self.__generator_types = {IPoissonSpikeGenerator:
                                  PyNNPoissonSpikeGenerator,
                                  IDCSource: PyNNDCSource,
                                  IACSource: PyNNACSource,
                                  INCSource: PyNNNCSource}
        self.__detector_types = {IIFCurrAlpha: PyNNIFCurrAlpha}
        self.generators = []
        self.detectors = []

    def register_spike_source(self, neurons, spike_generator_type, **kwargs):
        """
        Requests a communication object with the given spike generator type
        for the given set of neurons
        :param neurons: A reference to the neurons to which the spike generator
        should be connected
        :param spike_generator_type: A spike generator type (see documentation
        or a list of allowed values)
        :param kwargs: A dictionary of configuration parameters
        :return: A communication object
        """
        generator = self.__generator_types[spike_generator_type]()
        self.generators.append(generator)
        generator.connect(neurons, **kwargs)
        return generator

    def register_spike_sink(self, neurons, spike_detector_type, **kwargs):
        '''
        Requests a communication object with the given spike detector type
        for the given set of neurons
        :param neurons: A reference to the neurons which should be connected
        to the spike detector
        :param spike_detector_type: A spike detector type (see documentation
        for a list of allowed values)
        :param kwargs: A dictionary of configuration parameters
        :return: A Communication object
        '''
        detector = self.__detector_types[spike_detector_type]()
        self.detectors.append(detector)
        detector.connect(neurons, **kwargs)
        return detector

    def refresh_buffers(self, t):
        """
        Refreshes all detector buffers
        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the\
                                  concrete implementation")


class PyNNIFCurrAlpha(IIFCurrAlpha):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    def __init__(self, **kwargs):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to "infinity"
        is set to infinity by default in order to forbid the neuron to elicit
        spikes.
        """
        self.__cm = kwargs.get('cm', 1.0)
        self.__tau_m = kwargs.get('tau_m', 20.0)
        self.__tau_syn_E = kwargs.get('tau_syn_E', 0.5)
        self.__tau_syn_I = kwargs.get('tau_syn_I', 0.5)
        self.__v_rest = kwargs.get('v_rest', -65.0)
        self.__v_thresh = kwargs.get('v_thresh', float('inf'))
        self.__v_reset = kwargs.get('v_reset', -65.0)
        self.__tau_refrac = kwargs.get('tau_refrac', 1.0)
        self.__i_offset = kwargs.get('i_offset', 0.0)
        self.__voltage = kwargs.get('v_rest', -65.0)
        self.__cell = None

        self.create_device()
        self.start_record_voltage()

    def __get_voltage(self):
        return self.__cell.get_v()[-1, -1]

    voltage = property(__get_voltage)

    def __get_cm(self):
        return self.__cell.get('cm')[0]

    def __set_cm(self, cm):
        return self.__cell.set('cm', cm)

    cm = property(__get_cm, __set_cm)

    def __get_tau_m(self):
        return self.__cell.get('tau_m')[0]

    def __set_tau_m(self, tau_m):
        return self.__cell.set('tau_m', tau_m)

    tau_m = property(__get_tau_m, __set_tau_m)

    def __get_tau_syn_E(self):
        return self.__cell.get('tau_syn_E')[0]

    def __set_tau_syn_E(self, tau_syn_E):
        return self.__cell.set('tau_syn_E', tau_syn_E)

    tau_syn_E = property(__get_tau_syn_E, __set_tau_syn_E)

    def __get_tau_syn_I(self):
        return self.__cell.get('tau_syn_I')[0]

    def __set_tau_syn_I(self, tau_syn_I):
        return self.__cell.set('tau_syn_I', tau_syn_I)

    tau_syn_I = property(__get_tau_syn_I, __set_tau_syn_I)

    def __get_v_rest(self):
        return self.__cell.get('v_rest')[0]

    def __set_v_rest(self, v_rest):
        return self.__cell.set('v_rest', v_rest)

    v_rest = property(__get_v_rest, __set_v_rest)

    def __get_v_thresh(self):
        return self.__cell.get('v_thresh')[0]

    def __set_v_thresh(self, v_thresh):
        return self.__cell.set('v_thresh', v_thresh)

    v_thresh = property(__get_v_thresh, __set_v_thresh)

    def __get_v_reset(self):
        return self.__cell.get('v_reset')[0]

    def __set_v_reset(self, v_reset):
        return self.__cell.set('v_reset', v_reset)

    v_reset = property(__get_v_reset, __set_v_reset)

    def __get_tau_refrac(self):
        return self.__cell.get('tau_refrac')[0]

    def __set_tau_refrac(self, tau_refrac):
        return self.__cell.set('tau_refrac', tau_refrac)

    tau_refrac = property(__get_tau_refrac, __set_tau_refrac)

    def __get_i_offset(self):
        return self.__cell.get('i_offset')[0]

    def __set_i_offset(self, i_offset):
        return self.__cell.set('i_offset', i_offset)

    i_offset = property(__get_i_offset, __set_i_offset)

    def create_device(self):
        params = {'v_thresh': float('inf'),
                  'cm': self.__cm,
                  'tau_m': self.__tau_m,
                  'tau_syn_E': self.__tau_syn_E,
                  'tau_syn_I': self.__tau_syn_I,
                  'v_rest': self.__v_rest}
        self.__cell = sim.Population(1, sim.IF_curr_alpha, params)

    def start_record_voltage(self):
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


class PyNNPoissonSpikeGenerator(IPoissonSpikeGenerator):
    """
    Represents a Poisson spike generator
    """

    def __init__(self, **kwargs):
        """
        Initializes a Poisson spike generator.
        """
        self.__duration = kwargs.get('duration', float('inf'))
        self.__start = kwargs.get('start', 0.0)
        self.__rate = kwargs.get('rate', 10.0)
        self.__generator = None

        self.create_device()

    def __get_rate(self):
        return self.__generator.get('rate')[0]

    def __set_rate(self, rate):
        self.__generator.set('rate', rate)

    rate = property(__get_rate, __set_rate)

    def __get_duration(self):
        return self.__generator.get('duration')[0]

    def __set_duration(self, duration):
        self.__generator.set('duration', duration)

    duration = property(__get_duration, __set_duration)

    def __get_start(self):
        return self.__generator.get('start')[0]

    def __set_start(self, start):
        self.__generator.set('start', start)

    start = property(__get_start, __set_start)

    def create_device(self):
        params = {'duration': self.__duration,
                  'start': self.__start,
                  'rate': self.__rate}
        self.__generator = sim.Population(1, sim.SpikeSourcePoisson, params)

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
        return sim.Projection(presynaptic_population=self.__generator,
                              postsynaptic_population=neurons,
                              method=connector, source=source, target=target,
                              synapse_dynamics=synapse_dynamics, label=label,
                              rng=rng)


class PyNNDCSource(IDCSource):
    """
    Represents a direct current generator
    """

    def __init__(self, **kwargs):
        """
        Initializes a direct current generator.
        """
        self.__amplitude = kwargs.get('amplitude', 1.0)
        self.__start = kwargs.get('start', 0.0)
        self.__stop = kwargs.get('stop', None)
        self.__generator = None

        self.create_device()

    def __get_amplitude(self):
        return self.__generator.amplitude

    def __set_amplitude(self, amplitude):
        self.__generator.amplitude = amplitude

    amplitude = property(__get_amplitude, __set_amplitude)

    def __get_stop(self):
        return self.__generator.stop

    def __set_stop(self, stop):
        self.__generator.stop = stop

    stop = property(__get_stop, __set_stop)

    def __get_start(self):
        return self.__generator.start

    def __set_start(self, start):
        self.__generator.start = start

    start = property(__get_start, __set_start)

    def create_device(self):
        self.__generator = sim.DCSource(amplitude=self.__amplitude,
                                        start=self.__start, stop=self.__stop)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)


class PyNNACSource(IACSource):
    """
    Represents an alternating current generator
    """

    def __init__(self, **kwargs):
        """
        Initializes an alternating current generator.
        """
        self.__amplitude = kwargs.get('amplitude', 1.0)
        self.__offset = kwargs.get('offset', 0.0)
        self.__frequency = kwargs.get('frequency', 10.0)
        self.__phase = kwargs.get('phase', 0.0)
        self.__start = kwargs.get('start', 0.0)
        self.__stop = kwargs.get('stop', None)
        self.__generator = None

        self.create_device()

    def __get_amplitude(self):
        return self.__generator.amplitude

    def __set_amplitude(self, amplitude):
        self.__generator.amplitude = amplitude

    amplitude = property(__get_amplitude, __set_amplitude)

    def __get_offset(self):
        return self.__generator.offset

    def __set_offset(self, offset):
        self.__generator.offset = offset

    offset = property(__get_offset, __set_offset)

    def __get_frequency(self):
        return self.__generator.frequency

    def __set_frequency(self, frequency):
        self.__generator.frequency = frequency

    frequency = property(__get_frequency, __set_frequency)

    def __get_phase(self):
        return self.__generator.phase

    def __set_phase(self, phase):
        self.__generator.phase = phase

    phase = property(__get_phase, __set_phase)

    def create_device(self):
        self.__generator = sim.ACSource(amplitude=self.__amplitude,
                                        offset=self.__offset,
                                        frequency=self.__frequency,
                                        phase=self.__phase, start=self.__start,
                                        stop=self.__stop)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)


class PyNNNCSource(INCSource):
    """
    Represents a noisy current generator
    """

    def __init__(self, **kwargs):
        """
        Initializes a noisy current generator.
        """
        self.__mean = kwargs.get('mean', 0.0)
        self.__stdev = kwargs.get('stdev', 1.0)
        self.__dt = kwargs.get('dt', None)
        self.__start = kwargs.get('start', 0.0)
        self.__stop = kwargs.get('stop', None)
        self.__rng = kwargs.get('rng', None)
        self.__generator = None

        self.create_device()

    def __get_mean(self):
        return self.__generator.mean

    def __set_mean(self, mean):
        self.__generator.mean = mean

    mean = property(__get_mean, __set_mean)

    def __get_stop(self):
        return self.__generator.stop

    def __set_stop(self, stop):
        self.__generator.stop = stop

    stop = property(__get_stop, __set_stop)

    def __get_start(self):
        return self.__generator.start

    def __set_start(self, start):
        self.__generator.start = start

    start = property(__get_start, __set_start)

    def __get_stdev(self):
        return self.__generator.stdev

    def __set_stdev(self, stdev):
        self.__generator.stdev = stdev

    stdev = property(__get_stdev, __set_stdev)

    def __get_dt(self):
        return self.__generator.dt

    def __set_dt(self, dt):
        self.__generator.dt = dt

    dt = property(__get_dt, __set_dt)

    def __get_rng(self):
        return self.__generator.rng

    def __set_rng(self, rng):
        self.__generator.rng = rng

    rng = property(__get_rng, __set_rng)

    def create_device(self):
        self.__generator = sim.NoisyCurrentSource(
            mean=self.__mean, stdev=self.__stdev, dt=self.__dt,
            start=self.__start, stop=self.__stop, rng=self.__rng)

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.
        param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)


if __name__ == "__main__":
    '''
    PyNN Standard units: mV, nF, ms, nA, nS
    '''
    rank = sim.setup(timestep=0.1, min_delay=0.1, max_delay=4.0,
                     threads=1, rng_seeds=[23])

    print "Setting up process %d." % rank

    IFCELL = sim.Population(10, sim.IF_cond_exp, {'i_offset': 0.0,
                                                  'tau_refrac': 3.0,
                                                  'v_thresh': -51.0,
                                                  'tau_syn_E': 2.0,
                                                  'tau_syn_I': 5.0,
                                                  'v_rest': -65.0,
                                                  'v_reset': -70.0,
                                                  'e_rev_E': 0.,
                                                  'e_rev_I': -80.})

    IFCELL2 = sim.Population(10, sim.IF_cond_exp, {'i_offset': 0.0,
                                                   'tau_refrac': 10.0,
                                                   'v_thresh': -64.90,
                                                   'tau_syn_E': 2.0,
                                                   'tau_syn_I': 5.0,
                                                   'v_rest': -65.0,
                                                   'v_reset': -65.0,
                                                   'e_rev_E': 0.,
                                                   'e_rev_I': -80.})

    CONN = sim.AllToAllConnector(weights=2.0, delays=0.1)
    POISSON = PyNNPoissonSpikeGenerator()
    POISSON.connect(IFCELL2, connector=CONN)
    CELL = PyNNIFCurrAlpha()
    CELL.connect(IFCELL)
    DC = PyNNDCSource(amplitude=10.0)
    DC.connect(IFCELL)

    IFCELL[[0]].record_v()

    for i in xrange(100):
        print i+1
        sim.run(0.1)
        print CELL.voltage
        POISSON.rate = 500. * np.sin(0.002*i) + 500.
        print POISSON.rate
    print CELL.v_rest
    IFCELL[[0]].print_v("Results/IF_cond_exp_%s.v" % sim.__name__[5:])
    sim.end()
