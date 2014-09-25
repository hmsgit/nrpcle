"""
Test the mocked brain communication, control and
 devices
moduleauthor: Michael.Weber@fzi.de
"""

from python_cle.brainsim.BrainInterface import IIFCurrAlpha, ISpikeDetector, \
    IPoissonSpikeGenerator, IDCSource, IACSource, INCSource
from python_cle.mocks.brainsim.MockBrainCommunicationAdapter import \
    MockBrainCommunicationAdapter
from python_cle.mocks.brainsim.MockBrainControlAdapter import MockBrainControlAdapter
from python_cle.mocks.brainsim.devices.MockPoissonSpikeGenerator import \
    MockPoissonSpikeGenerator
from python_cle.mocks.brainsim.devices.MockDCSource import MockDCSource
from python_cle.mocks.brainsim.devices.MockACSource import MockACSource
from python_cle.mocks.brainsim.devices.MockNCSource import MockNCSource
from python_cle.mocks.brainsim.devices.MockIFCurrAlpha import MockIFCurrAlpha
from python_cle.mocks.brainsim.devices.MockSpikeDetector import MockSpikeDetector
import unittest

__author__ = 'MichaelWeber'


class MockBrainAdaptersTest(unittest.TestCase):
    """
    Tests the communication and control adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IPoissonSpikeGenerator: MockPoissonSpikeGenerator,
                     IDCSource: MockDCSource,
                     IACSource: MockACSource,
                     INCSource: MockNCSource,
                     IIFCurrAlpha: MockIFCurrAlpha,
                     ISpikeDetector: MockSpikeDetector}

    def setUp(self):
        """
        Instantiates the PyNN communication and control adapter
        """
        self.control = MockBrainControlAdapter()
        self.assertEqual(self.control.is_alive(), False)
        self.control.initialize(timestep=0.1,
                                min_delay=0.1,
                                max_delay=4.0,
                                num_threads=1)
        self.assertEqual(self.control.is_alive(), True)
        self.communicator = MockBrainCommunicationAdapter()
        self.neurons_cond = "4"
        self.neurons_curr = "8"
        self.two_neurons_pop_cond = [[1,2,3,4],[1,2,3,4]]
        self.two_neurons_pop_curr = [[5,6,7,8],[5,6,7,8]]

        self.assertEqual(self.communicator.is_initialized, False)
        self.assertEqual(self.communicator.detector_devices, [])
        self.assertEqual(self.communicator.generator_devices, [])

    def test_initialize(self):
        """
        Test the initialization of the Mock Brain adapters
        """
        self.communicator.initialize()
        self.assertEqual(self.communicator.is_initialized, True)

    def test_register_spike_source(self):
        """
        Tests the registration of the generator devices
        """
        device_0 = self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator)
        self.assertEqual(device_0, self.communicator.generator_devices[0])
        # self.assertIsInstance(device_0, PyNNPoissonSpikeGenerator)

        print("Poisson Spike Generator Rate (before): ",
              self.communicator.generator_devices[0].rate)
        self.communicator.generator_devices[0].rate = 2.0
        print("Poisson Spike Generator Rate (after): ",
              self.communicator.generator_devices[0].rate)

        device_1 = self.communicator.register_spike_source(
            self.neurons_cond, IDCSource)
        self.assertEqual(device_1, self.communicator.generator_devices[1])
        # self.assertIsInstance(device_1, PyNNDCSource)

        print("DC Amplitude (before): ",
              self.communicator.generator_devices[1].amplitude)
        self.communicator.generator_devices[1].amplitude = 2.0
        print("DC Amplitude (after): ",
              self.communicator.generator_devices[1].amplitude)

        device_2 = self.communicator.register_spike_source(
            self.neurons_cond, IACSource)
        self.assertEqual(device_2, self.communicator.generator_devices[2])
        # self.assertIsInstance(device_2, PyNNACSource)

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

        device_3 = self.communicator.register_spike_source(
            self.neurons_cond, INCSource)
        self.assertEqual(device_3, self.communicator.generator_devices[3])
        # self.assertIsInstance(device_3, PyNNNCSource)

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

        device_4 = self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator)
        self.assertEqual(device_4, self.communicator.generator_devices[4])
        # self.assertIsInstance(device_4, PyNNPoissonSpikeGenerator)

        device_5 = self.communicator.register_spike_source(
            self.two_neurons_pop_cond, IPoissonSpikeGenerator)
        self.assertEqual(device_5, self.communicator.generator_devices[5])
        # self.assertIsInstance(device_5, PyNNPoissonSpikeGenerator)

        device_6 = self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IPoissonSpikeGenerator)
        self.assertEqual(device_6, self.communicator.generator_devices[6])
        # self.assertIsInstance(device_6, PyNNPoissonSpikeGenerator)

        device_7 = self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator, target="inhibitory")
        self.assertEqual(device_7, self.communicator.generator_devices[7])
        # self.assertIsInstance(device_7, PyNNPoissonSpikeGenerator)

        device_8 = self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator, target="inhibitory")
        self.assertEqual(device_8, self.communicator.generator_devices[8])
        # self.assertIsInstance(device_8, PyNNPoissonSpikeGenerator)

    def test_register_spike_sink(self):
        """
        Tests the registration of the detector devices
        """
        device_0 = self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeDetector)
        self.assertEqual(device_0, self.communicator.detector_devices[0])
        # self.assertIsInstance(device_0, PyNNSpikeDetector)

        device_1 = self.communicator.register_spike_sink(
            self.neurons_cond, IIFCurrAlpha)
        self.assertEqual(device_1, self.communicator.detector_devices[1])
        # self.assertIsInstance(device_1, PyNNIFCurrAlpha)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device IFCurrAlpha): ",
              self.communicator.detector_devices[1].voltage)

        device_2 = self.communicator.register_spike_sink(
            self.neurons_curr, IIFCurrAlpha)
        self.assertEqual(device_2, self.communicator.detector_devices[2])
        # self.assertIsInstance(device_2, PyNNIFCurrAlpha)

        device_3 = self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, IIFCurrAlpha)
        self.assertEqual(device_3, self.communicator.detector_devices[3])
        # self.assertIsInstance(device_3, PyNNIFCurrAlpha)

        device_4 = self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, IIFCurrAlpha)
        self.assertEqual(device_4, self.communicator.detector_devices[4])
        # self.assertIsInstance(device_4, PyNNIFCurrAlpha)

        device_5 = self.communicator.register_spike_sink(
            self.neurons_curr, IIFCurrAlpha, target='inhibitory')
        self.assertEqual(device_5, self.communicator.detector_devices[5])
        # self.assertIsInstance(device_5, PyNNIFCurrAlpha)

    def test_refresh_buffers(self):
        """
        Tests the refresh_buffers function
        """
        device_0 = self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeDetector)
        self.assertEqual(device_0, self.communicator.detector_devices[0])
        #self.assertIsInstance(device_0, SpikeDetector)

        device_1 = self.communicator.register_spike_sink(
            self.neurons_cond, IIFCurrAlpha)
        self.assertEqual(device_1, self.communicator.detector_devices[1])
        # self.assertIsInstance(device_1, PyNNIFCurrAlpha)

        time = 0.2
        self.control.run_step(0.1)
        self.communicator.refresh_buffers(time)
        self.control.shutdown()
        self.assertEqual(self.communicator.refreshed_at[0], time)


if __name__ == "__main__":
    unittest.main()
