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
Test the PyNN communication, PyNN control and
PyNN devices
moduleauthor: probst@fzi.de
"""

import unittest

import numpy as np

import pyNN.nest as sim
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder, \
    IPoissonSpikeGenerator, IDCSource, IACSource, INCSource, \
    IPopulationRate, IFixedSpikeGenerator, ILeakyIntegratorAlpha, \
    ILeakyIntegratorExp, IDeviceGroup
from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter
from hbp_nrp_cle.brainsim.pynn_nest.PyNNNestCommunicationAdapter import \
    PyNNNestCommunicationAdapter
from mock import mock_open, patch, Mock
from testfixtures import log_capture, LogCapture
import hbp_nrp_cle.tf_framework as tf_framework
import hbp_nrp_cle.brainsim.config as brainconfig

__author__ = 'DimitriProbst'


class PyNNAdaptersTest(unittest.TestCase):
    """
    Tests the communication and control adapter to the neuronal simulator
    """

    def setUp(self):
        """
        Instantiates the PyNN communication and control adapter
        """
        brainconfig.rng_seed = 123456
        with LogCapture(('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter',
                        'hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter',
                         'hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter')) as log_capt:
            self.control = PyNNControlAdapter(sim)
            self.assertEqual(self.control.is_alive(), False)
            self.control.initialize(timestep=0.1,
                                    min_delay=0.1,
                                    max_delay=4.0,
                                    num_threads=1)
            self.control.initialize(timestep=0.1,
                                    min_delay=0.1,
                                    max_delay=4.0,
                                    num_threads=1)
            self.communicator = PyNNNestCommunicationAdapter()
            self.neurons_cond = sim.Population(10, sim.IF_cond_exp())
            self.neurons_curr = sim.Population(10, sim.IF_curr_exp())
            self.two_neurons_pop_cond = [sim.Population(10, sim.IF_cond_exp()),
                                         sim.Population(10, sim.IF_cond_exp())]
            self.two_neurons_pop_curr = [sim.Population(10, sim.IF_curr_exp()),
                                         sim.Population(10, sim.IF_curr_exp())]
            self.three_neurons_pop_cond = [sim.Population(10, sim.IF_cond_exp()),
                                           sim.Population(10, sim.IF_cond_exp()),
                                           sim.Population(10, sim.IF_cond_exp())]

            self.assertEqual(self.communicator.is_initialized, False)
            self.assertEqual(self.communicator.detector_devices, [])
            self.assertEqual(self.communicator.generator_devices, [])
        log_capt.check(('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter', 'INFO',
                        'neuronal simulator initialized'),
                       ('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter', 'WARNING',
                        'trying to initialize an already initialized controller'))

    @log_capture('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter',
                 'hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter')
    def test_reset(self, logcapture):
        """
        Test the reset functionality. The reset must not unload the brain, therefore the neurons
        that have been setup before must still exist after the reset.
        """
        self.control.reset()

        population = sim.Population(10, sim.IF_cond_exp())
        # As PyNN >= 0.8 creates a multimeter and a spike_recorder per population, the ID of the
        # 10th neuron of the 10th population (9 are created during setUp) must be 120
        self.assertEqual(population.all_cells[9], 120)
        logcapture.check(('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter', 'INFO',
                          'neuronal simulator reset'))

    @log_capture('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter',
                 'hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter')
    def test_initialize(self, logcapture):
        """
        Test the initialization of the PyNN adapters
        """
        self.communicator.initialize()
        self.assertEqual(self.communicator.is_initialized, True)
        logcapture.check(('hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter', 'INFO',
                          'PyNN communication adapter initialized'))

    @log_capture('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter',
                 'hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter',
                 'hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter')
    def test_register_spike_source(self, logcapture):
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

        self.communicator.register_spike_source(self.neurons_cond, INCSource)
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
            self.two_neurons_pop_cond, IPoissonSpikeGenerator,
                receptor_type=['excitatory', 'inhibitory'])
        self.assertIsInstance(self.communicator.generator_devices[5], IDeviceGroup)
        self.assertEqual(len(self.communicator.generator_devices[5]), 2)
        self.assertIsInstance(self.communicator.generator_devices[5][0], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[5][1], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[6], IDeviceGroup)
        self.assertIsInstance(self.communicator.generator_devices[6][0], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[6][1], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator, receptor_type="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[7], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator, receptor_type="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[8], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_cond, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[9], IFixedSpikeGenerator)

        print("Fixed Frequency Spike Generator Rate (before): ",
              self.communicator.generator_devices[9].rate)
        self.communicator.generator_devices[9].rate = 2.0
        print("Fixed Frequency Spike Generator Rate (after): ",
              self.communicator.generator_devices[9].rate)

        self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[10], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_cond, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[11], IDeviceGroup)
        self.assertIsInstance(self.communicator.generator_devices[11][0], IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[11][1], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[12], IDeviceGroup)
        self.assertIsInstance(self.communicator.generator_devices[12][0], IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[12][1], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_cond, IFixedSpikeGenerator, receptor_type="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[13], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator, receptor_type="inhibitory")
        self.assertIsInstance(self.communicator.generator_devices[14], IFixedSpikeGenerator)

        group = self.communicator.register_spike_source(
            self.three_neurons_pop_cond, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[15], IDeviceGroup)
        self.assertIsInstance(self.communicator.generator_devices[15][0], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[15][1], IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[15][2], IPoissonSpikeGenerator)

        self.assertIsInstance(group, IDeviceGroup)

        np.testing.assert_array_equal(group.rate, np.array([0.0, 0.0, 0.0]))
        print "Poisson generator group: rates (before)", group.rate
        group.rate = np.array([3.0, 4.0, 5.0])
        np.testing.assert_array_equal(group.rate, np.array([3.0, 4.0, 5.0]))
        print "Poisson generator group: change all rates separaterly (after)", group.rate
        group.rate = 2.0
        np.testing.assert_array_equal(group.rate, np.array([2.0, 2.0, 2.0]))
        print "Poisson generator group: set all rates to the same value (after)", group.rate
        group[1].rate = 3.0
        np.testing.assert_array_equal(group.rate, np.array([2.0, 3.0, 2.0]))
        print "Poisson generator group: set second rate to 3.0 Hz (after)", group.rate
        group[0:2].rate = np.array([5.0, 5.0])
        np.testing.assert_array_equal(group.rate, np.array([5.0, 5.0, 2.0]))
        print "Poisson generator group: change the rates between in slice (0, 2, None) (after)", \
            group.rate



        self.communicator.register_spike_source(
            self.neurons_cond, IDCSource, parrot=True)
        self.assertIsInstance(self.communicator.generator_devices[16], IDCSource)

        print("DC Amplitude (before): ",
              self.communicator.generator_devices[16].amplitude)
        self.communicator.generator_devices[16].amplitude = 2.0
        print("DC Amplitude (after): ",
              self.communicator.generator_devices[16].amplitude)

        with self.assertRaises(TypeError):
            print group['test'].rate
        with self.assertRaises(KeyError):
            print group.voltage
        with self.assertRaises(KeyError):
            print group[0:3].voltage
        logcapture.check(('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IDCSource\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IACSource\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.INCSource\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IFixedSpikeGenerator\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPoissonSpikeGenerator\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IDCSource\'>" \
requested (device)'))

        self.communicator.shutdown()
        self.assertEqual(0, len(self.communicator.detector_devices))
        self.assertEqual(0, len(self.communicator.generator_devices))
        self.assertEqual(0, len(self.communicator.refreshable_devices))
        self.assertEqual(0, len(self.communicator.finalizable_devices))

    def test_unregister_spike_source(self):
        """
        Tests the deregistration of the generator __devices
        """
        poisson = self.communicator.register_spike_source(self.neurons_cond,
                                                          IPoissonSpikeGenerator)
        self.communicator.unregister_spike_source(poisson)
        self.assertEquals(poisson._generator, None)

        dc = self.communicator.register_spike_source(self.neurons_cond, IDCSource)
        self.communicator.unregister_spike_source(dc)
        self.assertEquals(dc._generator, None)

        dc2 = self.communicator.register_spike_source(self.neurons_cond, IDCSource, parrot=True)
        self.communicator.unregister_spike_source(dc2)
        self.assertEquals(dc2._generator, None)

        ac = self.communicator.register_spike_source(self.neurons_cond, IACSource)
        self.communicator.unregister_spike_source(ac)
        self.assertEquals(ac._generator, None)

        nc = self.communicator.register_spike_source(self.neurons_cond, INCSource)
        self.communicator.unregister_spike_source(nc)
        self.assertEquals(nc._generator, None)

        fixed = self.communicator.register_spike_source(self.neurons_cond, IFixedSpikeGenerator)
        self.communicator.unregister_spike_source(fixed)
        self.assertEquals(fixed._generator, None)
        self.assertEquals(fixed._currentsource, None)

        # test that double remove does not cause a failure
        self.communicator.unregister_spike_source(fixed)

    @log_capture('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter',
                 'hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter',
                 'hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter')
    def test_register_spike_sink(self, logcapture):
        """
        Tests the registration of the detector __devices
        """
        self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeRecorder)
        self.assertIsInstance(self.communicator.detector_devices[0], ISpikeRecorder)
        self.assertIsInstance(self.communicator.refreshable_devices[0], ISpikeRecorder)

        self.communicator.register_spike_sink(self.neurons_cond, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[1], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[1], ILeakyIntegratorAlpha)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device IFCurrAlpha): ",
              self.communicator.detector_devices[1].voltage)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[2], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[2], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[3], IDeviceGroup)
        self.assertIsInstance(self.communicator.refreshable_devices[3], IDeviceGroup)

        self.assertIsInstance(self.communicator.detector_devices[3][0], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[3][0], ILeakyIntegratorAlpha)

        self.assertIsInstance(self.communicator.detector_devices[3][1], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[3][1], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.detector_devices[4], IDeviceGroup)
        self.assertIsInstance(self.communicator.detector_devices[4], IDeviceGroup)

        self.assertIsInstance(self.communicator.detector_devices[4][0], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[4][0], ILeakyIntegratorAlpha)

        self.assertIsInstance(self.communicator.detector_devices[4][1], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[4][1], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorAlpha, receptor_type='inhibitory')
        self.assertIsInstance(self.communicator.detector_devices[5], ILeakyIntegratorAlpha)
        self.assertIsInstance(self.communicator.refreshable_devices[5], ILeakyIntegratorAlpha)

        self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[6], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[6], ILeakyIntegratorExp)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device IFCurrExp): ",
              self.communicator.detector_devices[6].voltage)

        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[7], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[7], ILeakyIntegratorExp)


        self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[8], IDeviceGroup)
        self.assertIsInstance(self.communicator.refreshable_devices[8], IDeviceGroup)

        self.assertIsInstance(self.communicator.detector_devices[8][0], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[8][0], ILeakyIntegratorExp)

        self.assertIsInstance(self.communicator.detector_devices[8][1], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[8][1], ILeakyIntegratorExp)

        self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.detector_devices[9], IDeviceGroup)
        self.assertIsInstance(self.communicator.refreshable_devices[9], IDeviceGroup)

        self.assertIsInstance(self.communicator.detector_devices[9][0], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[9][0], ILeakyIntegratorExp)

        self.assertIsInstance(self.communicator.detector_devices[9][1], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[9][1], ILeakyIntegratorExp)


        self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorExp, receptor_type='inhibitory')
        self.assertIsInstance(self.communicator.detector_devices[10], ILeakyIntegratorExp)
        self.assertIsInstance(self.communicator.refreshable_devices[10], ILeakyIntegratorExp)


        self.communicator.register_spike_sink(
            self.neurons_curr, IPopulationRate)
        self.assertIsInstance(self.communicator.detector_devices[11], IPopulationRate)
        self.assertIsInstance(self.communicator.refreshable_devices[11], IPopulationRate)

        #check communicator lists length
        self.assertEquals(len(self.communicator.detector_devices), 12)
        self.assertEquals(len(self.communicator.refreshable_devices), 12)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device PopulationRate): ",
              self.communicator.detector_devices[11].rate)
        logcapture.check(('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ISpikeRecorder\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with \
type "<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorAlpha\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp\'>" \
requested (device group)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.ILeakyIntegratorExp\'>" \
requested (device)'),
                         ('hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter', 'INFO',
                          'Communication object with type \
"<class \'hbp_nrp_cle.brainsim.BrainInterface.IPopulationRate\'>" \
requested (device)'))

        self.communicator.shutdown()
        self.assertEqual(0, len(self.communicator.detector_devices))
        self.assertEqual(0, len(self.communicator.generator_devices))
        self.assertEqual(0, len(self.communicator.refreshable_devices))
        self.assertEqual(0, len(self.communicator.finalizable_devices))

    def test_refresh_buffers(self):
        """
        Tests the refresh_buffers function
        """
        self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeRecorder)
        self.assertIsInstance(self.communicator.detector_devices[0], ISpikeRecorder)

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

    @patch("hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter.BrainLoader")
    def test_load_brain(self, loader):
        with patch('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter.open',
                   mock_open(read_data='some python code'),
                   create=True):
            slice1 = {'from': 1, 'to': 2, 'step': 3}
            slice2 = {'from': 4, 'to': 5, 'step': None}

            populations_mixed = {
                'slice_1': slice1, 'slice_2': slice(4, 5),
                'list_1': [1, 2, 3]
            }
            self.control.load_brain("foo.py", **populations_mixed)

            loader.load_py_network.assert_called_with("foo.py")
            loader.setup_access_to_population.assert_called_once()
            self.assertTrue(loader.load_py_network.called)
            self.assertEqual(tf_framework.config.brain_source, 'some python code')

            populations_json = {
                'slice_1': slice1, 'slice_2': slice2,
                'list_1': [1, 2, 3]
            }
            self.assertEqual(
              tf_framework.config.brain_populations,
              populations_json
            )
            loader.load_py_network.reset_mock()

    @patch("hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter.BrainLoader")
    def test_load_brain_no_population(self, _loader):
        with patch('hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter.open',
                   mock_open(read_data='some python code'),
                   create=True):

            current_populations = {
                'slice_1': {'from': 1, 'to': 2, 'step': 3}
            }

            brain_pop_bak = tf_framework.config.brain_populations
            tf_framework.config.brain_populations = current_populations

            with patch.object(self.control, 'load_populations') as load_pops_patch:
                self.control.load_brain("foo.py")
                load_pops_patch.assert_called_with(**current_populations)

            tf_framework.config.brain_populations = brain_pop_bak

    def test_populations_no_brain_loaded(self):
        populations = {'slice_1': {'from': 1, 'to': 2, 'step': 3}}

        brain_root_bak = tf_framework.config.brain_root
        tf_framework.config.brain_root = None

        # brain_root is None
        with self.assertRaises(Exception):
            self.control.load_populations(**populations)

        # brain_root.circuit missing
        tf_framework.config.brain_root = Mock(spec=[])

        with self.assertRaises(AttributeError):
            self.control.load_populations(**populations)

        tf_framework.config.brain_root = brain_root_bak

    def test_populations_using_json_slice(self):
        slice1 = {'from': 1, 'to': 2, 'step': 3}
        slice2 = {'from': 1, 'to': 2, 'step': None}
        populations_json_slice = {
          'slice_1': slice1, 'slice_2': slice2,
          'list_1': [1, 2, 3]
        }
        populations_python_slice = {
          'slice_1': slice(1, 2, 3), 'slice_2': slice(1, 2),
          'list_1': [1, 2, 3]
        }
        self.assertEqual(
          self.control.populations_using_json_slice(populations_json_slice),
          populations_json_slice
        )

        self.assertEqual(
          self.control.populations_using_json_slice(populations_python_slice),
          populations_json_slice
        )

    def test_populations_using_python_slice(self):
        slice1 = {'from': 1, 'to': 2, 'step': 3}
        slice2 = {'from': 1, 'to': 2}
        populations_json_slice = {
          'population_1': 1, 'population_2': 2,
          'slice_1': slice1, 'slice_2': slice2,
          'list_1': [1, 2, 3]
        }
        populations_python_slice = {
          'population_1': 1, 'population_2': 2,
          'slice_1': slice(1, 2, 3), 'slice_2': slice(1, 2),
          'list_1': [1, 2, 3]
        }
        self.assertEqual(
          self.control.populations_using_python_slice(populations_json_slice),
          populations_python_slice
        )

        self.assertEqual(
          self.control.populations_using_python_slice(populations_python_slice),
          populations_python_slice
        )

    def test_shutdown(self):

        self.control.shutdown()

        # communicator's lists must be empty
        self.assertEquals(len(self.communicator.detector_devices), 0)
        self.assertEquals(len(self.communicator.refreshable_devices), 0)
        self.assertEquals(len(self.communicator.finalizable_devices), 0)

        self.assertFalse(self.control.is_initialized)
        self.assertFalse(self.communicator.is_initialized)

    def tearDown(self):
        """
        Shuts down the Brain control
        """
        self.control.shutdown()


if __name__ == "__main__":
    unittest.main()
