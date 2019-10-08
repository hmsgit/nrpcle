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

import unittest
import nest
from hbp_nrp_cle.brainsim.nest.NestControlAdapter import NestControlAdapter
from hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter import NestCommunicationAdapter
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder, \
    IPoissonSpikeGenerator, IDCSource, IACSource, INCSource, \
    IPopulationRate, IFixedSpikeGenerator, ILeakyIntegratorAlpha, \
    ILeakyIntegratorExp, IDeviceGroup
from testfixtures import log_capture
import numpy as np


class NestCommunicationAdapterTest(unittest.TestCase):

    def setUp(self):

        self.control = NestControlAdapter(nest)
        self.control.initialize()

        self.communicator = NestCommunicationAdapter()

        self.neurons_cond = nest.Create('iaf_cond_exp', 10)
        self.neurons_curr = nest.Create('iaf_psc_exp', 10)
        self.two_neurons_pop_cond = [nest.Create('iaf_cond_exp', 10),
                                     nest.Create('iaf_cond_exp', 10)]
        self.two_neurons_pop_curr = [nest.Create('iaf_psc_exp', 10),
                                     nest.Create('iaf_psc_exp', 10)]
        self.three_neurons_pop_cond = [nest.Create('iaf_cond_exp', 10),
                                       nest.Create('iaf_cond_exp', 10),
                                       nest.Create('iaf_cond_exp', 10)]

        self.assertEqual(self.communicator.is_initialized, False)
        self.assertEqual(self.communicator.detector_devices, [])
        self.assertEqual(self.communicator.generator_devices, [])

    @log_capture('hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter')
    def test_initialize(self, logcapture):
        """
        Test the initialization of the Nest adapter
        """
        self.communicator.initialize()
        self.assertEqual(self.communicator.is_initialized, True)
        logcapture.check(('hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter', 'INFO',
                          'NEST communication adapter initialized'))

    @log_capture('hbp_nrp_cle.brainsim.nest.NestControlAdapter',
                 'hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter',
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
            self.two_neurons_pop_cond, IPoissonSpikeGenerator)
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
            self.neurons_cond, IPoissonSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[7], IPoissonSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator)
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
            self.neurons_cond, IFixedSpikeGenerator)
        self.assertIsInstance(self.communicator.generator_devices[13], IFixedSpikeGenerator)

        self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator)
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
            self.neurons_cond, IDCSource)
        self.assertIsInstance(self.communicator.generator_devices[16], IDCSource)

        print("DC Amplitude (before): ",
              self.communicator.generator_devices[16].amplitude)
        self.communicator.generator_devices[16].amplitude = 2.0
        print("DC Amplitude (after): ",
              self.communicator.generator_devices[16].amplitude)

        with self.assertRaises(TypeError):
            print group['test'].rate
        with self.assertRaises(AttributeError):
            print group.voltage
        with self.assertRaises(AttributeError):
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

    @log_capture('hbp_nrp_cle.brainsim.nest.NestControlAdapter',
                 'hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter',
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

        # check communicator lists length
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

    def test_shutdown(self):

        self.control.shutdown()

        # communicator's lists must be empty
        self.assertEquals(len(self.communicator.detector_devices), 0)
        self.assertEquals(len(self.communicator.refreshable_devices), 0)
        self.assertEquals(len(self.communicator.finalizable_devices), 0)

        self.assertFalse(self.control.is_initialized)
        self.assertFalse(self.communicator.is_initialized)
