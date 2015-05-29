"""
Test for single functions of the bibi configuration script
"""

__author__ = 'GeorgHinkel'

import hbp_nrp_cle.bibi_config.bibi_configuration_script as bibi
import hbp_nrp_cle.bibi_config.generated.generated_bibi_api as api
import unittest


class TestScript(unittest.TestCase):
    def test_is_not_none(self):
        self.assertTrue(bibi.is_not_none("Foo"))
        self.assertFalse(bibi.is_not_none(None))

    def test_remove_extension(self):
        self.assertEqual(bibi.remove_extension("myFile.txt"), "myFile")
        self.assertEqual(bibi.remove_extension("foo.bar.txt"), "foo.bar")
        self.assertEqual(bibi.remove_extension("foo/bar.txt"), "foo/bar")

    def test_print_expression(self):
        self.assertRaises(Exception, bibi.print_expression, "foo")

    def test_monitoring_population_rate(self):
        monitor = api.Neuron2Monitor("testMonitor",
                                     device=api.DeviceChannel("PopulationRate", "neurons",
                                                              api.List("pop", [1, 2, 3])))
        self.assertEqual(bibi.get_monitoring_type(monitor), "cle_ros_msgs.msg.SpikeRate")
        self.assertEqual(bibi.get_monitoring_topic(monitor), "/monitor/population_rate")

        impl = 'cle_ros_msgs.msg.SpikeRate(t, neurons.rate, ' \
               '"testMonitor")'
        self.assertEqual(bibi.get_monitoring_impl(monitor), impl)

    def test_monitoring_spike_detector(self):
        monitor = api.Neuron2Monitor("testMonitor",
                                     device=api.DeviceChannel("SpikeRecorder", "neurons",
                                                              api.List("pop", [1, 2, 3])))
        self.assertEqual(bibi.get_monitoring_type(monitor), "cle_ros_msgs.msg.SpikeEvent")
        self.assertEqual(bibi.get_monitoring_topic(monitor), "/monitor/spike_recorder")

        impl = 'monitoring.create_spike_recorder_message(t, 3, neurons.times, "testMonitor")'
        self.assertEqual(bibi.get_monitoring_impl(monitor), impl)

    def test_get_neuron_count(self):
        self.assertEqual(3, bibi.get_neuron_count(api.List("pop", [1, 2, 3])))
        self.assertEqual(1, bibi.get_neuron_count(api.Index("pop")))
        self.assertEqual(4, bibi.get_neuron_count(api.Range("pop", from_=3, to=7)))
        self.assertEqual(2, bibi.get_neuron_count(api.Range("pop", from_=0, to=5, step=2)))


if __name__ == '__main__':
    unittest.main()