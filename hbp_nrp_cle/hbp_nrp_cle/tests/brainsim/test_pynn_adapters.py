"""
Test the PyNN communication, PyNN control and
PyNN devices
moduleauthor: probst@fzi.de
"""

from hbp_nrp_cle.brainsim.BrainInterface import ISpikeDetector, \
    IPoissonSpikeGenerator, IDCSource, IACSource, INCSource, \
    IPopulationRate, IFixedSpikeGenerator, ILeakyIntegratorAlpha, \
    ILeakyIntegratorExp, IDeviceGroup
from hbp_nrp_cle.brainsim.PyNNCommunicationAdapter import \
    PyNNCommunicationAdapter
from hbp_nrp_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
import pyNN.nest as sim
import unittest
import numpy as np

__author__ = 'DimitriProbst'


class PyNNAdaptersTest(unittest.TestCase):
    """
    Tests the communication and control adapter to the neuronal simulator
    """

    def setUp(self):
        """
        Instantiates the PyNN communication and control adapter
        """
        self.control = PyNNControlAdapter()
        self.assertEqual(self.control.is_alive(), False)
        self.control.initialize(timestep=0.1,
                                min_delay=0.1,
                                max_delay=4.0,
                                num_threads=1)
        self.assertEqual(self.control.is_alive(), True)
        self.communicator = PyNNCommunicationAdapter()
        self.neurons_cond = sim.Population(10, sim.IF_cond_exp)
        self.neurons_curr = sim.Population(10, sim.IF_curr_exp)
        self.two_neurons_pop_cond = [sim.Population(10, sim.IF_cond_exp),
                                     sim.Population(10, sim.IF_cond_exp)]
        self.two_neurons_pop_curr = [sim.Population(10, sim.IF_curr_exp),
                                     sim.Population(10, sim.IF_curr_exp)]
        self.three_neurons_pop_cond = [sim.Population(10, sim.IF_cond_exp),
                                       sim.Population(10, sim.IF_cond_exp),
                                       sim.Population(10, sim.IF_cond_exp)]

        self.assertEqual(self.communicator.is_initialized, False)
        self.assertEqual(self.communicator.detector_devices, [])
        self.assertEqual(self.communicator.generator_devices, [])

    def test_reset(self):
        """
        Test the reset functionality
        """
        self.control.reset()
        population = sim.Population(10, sim.IF_cond_exp)
        self.assertEqual(population.all_cells[9], 10)

    def test_initialize(self):
        """
        Test the initialization of the PyNN adapters
        """
        self.communicator.initialize()
        self.assertEqual(self.communicator.is_initialized, True)

    def test_register_spike_source(self):
        """
        Tests the registration of the generator __devices
        """
        self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[0], IPoissonSpikeGenerator)

        print("Poisson Spike Generator Rate (before): ",
              self.communicator.generator_devices[0].rate)
        self.communicator.generator_devices[0].rate = 2.0
        print("Poisson Spike Generator Rate (after): ",
              self.communicator.generator_devices[0].rate)

        self.communicator.register_spike_source(
            self.neurons_cond, IDCSource)
        self.assertIsInstance(self.communicator.generator_devices[1], IDCSource)

        print("DC Amplitude (before): ",
              self.communicator.generator_devices[1].amplitude)
        self.communicator.generator_devices[1].amplitude = 2.0
        print("DC Amplitude (after): ",
              self.communicator.generator_devices[1].amplitude)

        self.communicator.register_spike_source(
            self.neurons_cond, IACSource)
        self.assertIsInstance(self.communicator.generator_devices[2], IACSource)

        print("AC Amplitude (before): ",
              self.communicator.generator_devices[2].amplitude)
        print("AC Offset (before): ",
              self.communicator.generator_devices[2].offset)
        print("AC Frequency (before): ",
              self.communicator.generator_devices[2].frequency)
        print("AC Phase (before): ",
              self.communicator.generator_devices[2].phase)
        self.communicator.generator_devices[2].amplitude = 2.0
        self.communicator.generator_devices[2].offset = 2.0
        self.communicator.generator_devices[2].frequency = 2.0
        self.communicator.generator_devices[2].phase = 2.0
        print("AC Amplitude (after): ",
              self.communicator.generator_devices[2].amplitude)
        print("AC Offset (after): ",
              self.communicator.generator_devices[2].offset)
        print("AC Frequency (after): ",
              self.communicator.generator_devices[2].frequency)
        print("AC Phase (after): ",
              self.communicator.generator_devices[2].phase)

        self.communicator.register_spike_source(
            self.neurons_cond, INCSource)
        self.assertIsInstance(self.communicator.generator_devices[3], INCSource)

        print("NC Mean (before): ",
              self.communicator.generator_devices[3].mean)
        print("NC Standard Deviation (before): ",
              self.communicator.generator_devices[3].stdev)
        self.communicator.generator_devices[3].mean = 2.0
        self.communicator.generator_devices[3].stdev = 2.0
        print("NC Mean (after): ",
              self.communicator.generator_devices[3].mean)
        print("NC Standard Deviation (after): ",
              self.communicator.generator_devices[3].stdev)

        self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[4], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_cond, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[5], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[6], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[7], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[8], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator, target="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[9], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator, target="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[10], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_cond, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[11], IFixedSpikeGenerator)

        print("Fixed Frequency Spike Generator Rate (before): ",
              self.communicator.generator_devices[11].rate)
        self.communicator.generator_devices[11].rate = 2.0
        print("Fixed Frequency Spike Generator Rate (after): ",
              self.communicator.generator_devices[11].rate)

        self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[12], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_cond, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[13], IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[14], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[15], IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[16], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_cond, IFixedSpikeGenerator, target="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[17], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator, target="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[18], IFixedSpikeGenerator)

        group = self.communicator.register_spike_source(
            self.three_neurons_pop_cond, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[19], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[20], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[21], IPoissonSpikeGenerator)

        self.assertIsInstance(group, IDeviceGroup)

        np.testing.assert_array_equal(group.rate, np.array([0.0, 0.0, 0.0]))
        print "Poissong generator group: rates (before)", group.rate
        group.rate = np.array([3.0, 4.0, 5.0])
        np.testing.assert_array_equal(group.rate, np.array([3.0, 4.0, 5.0]))
        print "Poissong generator group: change all rates separaterly (after)", group.rate
        group.rate = 2.0
        np.testing.assert_array_equal(group.rate, np.array([2.0, 2.0, 2.0]))
        print "Poissong generator group: set all rates to the same value (after)", group.rate
        group[1].rate = 3.0
        np.testing.assert_array_equal(group.rate, np.array([2.0, 3.0, 2.0]))
        print "Poissong generator group: set second rate to 3.0 Hz (after)", group.rate
        group[0:2].rate = np.array([5.0, 5.0])
        np.testing.assert_array_equal(group.rate, np.array([5.0, 5.0, 2.0]))
        print "Poissong generator group: change the rates between in slice (0, 2, None) (after)", \
            group.rate

        with self.assertRaises(TypeError):
            print group['test'].rate
        with self.assertRaises(AttributeError):
            print group.voltage
        with self.assertRaises(AttributeError):
            print group[0:3].voltage

    def test_register_spike_sink(self):
        """
        Tests the registration of the detector __devices
        """
        self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeDetector)
        self.assertIsInstance(self.communicator.detector_devices[0], ISpikeDetector)

        self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[1], ILeakyIntegratorAlpha)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device IFCurrAlpha): ",
              self.communicator.detector_devices[1].voltage)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[2], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[3], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[4], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[5], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[6], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorAlpha, target='inhibitory')
        self.assertIsInstance(self.communicator.detector_devices[7], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[8], ILeakyIntegratorExp)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device IFCurrExp): ",
              self.communicator.detector_devices[6].voltage)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[9], ILeakyIntegratorExp)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[10], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[11], ILeakyIntegratorExp)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[12], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[13], ILeakyIntegratorExp)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorExp, target='inhibitory')
        self.assertIsInstance(self.communicator.detector_devices[14], ILeakyIntegratorExp)

        self.communicator.register_spike_sink(
            self.neurons_curr, IPopulationRate)
        self.assertIsInstance(self.communicator.detector_devices[15], IPopulationRate)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device PopulationRate): ",
              self.communicator.detector_devices[15].rate)

    def test_refresh_buffers(self):
        """
        Tests the refresh_buffers function
        """
        self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeDetector)
        self.assertIsInstance(self.communicator.detector_devices[0], ISpikeDetector)

        self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[1], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[2], ILeakyIntegratorExp)

        self.communicator.register_spike_sink(
            self.neurons_cond, IPopulationRate)
        self.assertIsInstance(self.communicator.detector_devices[3], IPopulationRate)

        time = 0.2
        self.control.run_step(0.1)
        self.communicator.refresh_buffers(time)

    def tearDown(self):
        """
        Shuts down the Brain control
        """
        self.control.shutdown()


if __name__ == "__main__":
    unittest.main()
