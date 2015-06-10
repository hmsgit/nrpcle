"""
Test for single functions of the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.bibi_config.bibi_configuration_script import *
import hbp_nrp_cle.bibi_config.generated.generated_bibi_api as api
import unittest


class TestScript(unittest.TestCase):
    def test_is_not_none(self):
        self.assertTrue(is_not_none("Foo"))
        self.assertFalse(is_not_none(None))

    def test_remove_extension(self):
        self.assertEqual(remove_extension("myFile.txt"), "myFile")
        self.assertEqual(remove_extension("foo.bar.txt"), "foo.bar")
        self.assertEqual(remove_extension("foo/bar.txt"), "foo/bar")

    def test_print_expression_raises(self):
        self.assertRaises(Exception, print_expression, "foo")

    def test_monitoring_population_rate(self):
        monitor = api.Neuron2Monitor("testMonitor",
                                     device=api.DeviceChannel("PopulationRate", "neurons",
                                                              api.List("pop", [1, 2, 3])))
        self.assertEqual(get_monitoring_type(monitor), "cle_ros_msgs.msg.SpikeRate")
        self.assertEqual(get_monitoring_topic(monitor), "/monitor/population_rate")

        impl = 'cle_ros_msgs.msg.SpikeRate(t, neurons.rate, ' \
               '"testMonitor")'
        self.assertEqual(get_monitoring_impl(monitor), impl)

    def test_monitoring_spike_detector(self):
        monitor = api.Neuron2Monitor("testMonitor",
                                     device=api.DeviceChannel("SpikeRecorder", "neurons",
                                                              api.List("pop", [1, 2, 3])))
        self.assertEqual(get_monitoring_type(monitor), "cle_ros_msgs.msg.SpikeEvent")
        self.assertEqual(get_monitoring_topic(monitor), "/monitor/spike_recorder")

        impl = 'monitoring.create_spike_recorder_message(t, 3, neurons.times, "testMonitor")'
        self.assertEqual(get_monitoring_impl(monitor), impl)

    def test_get_neuron_count(self):
        self.assertEqual(3, get_neuron_count(api.List("pop", [1, 2, 3])))
        self.assertEqual(1, get_neuron_count(api.Index("pop")))
        self.assertEqual(4, get_neuron_count(api.Range("pop", from_=3, to=7)))
        self.assertEqual(2, get_neuron_count(api.Range("pop", from_=0, to=5, step=2)))
        self.assertEqual(42, get_neuron_count(api.Population("pop", 42)))

    def test_get_device_name(self):
        __device_types = {'ACSource': 'ac_source', 'DCSource': 'dc_source',
                          'FixedFrequency': 'fixed_frequency',
                          'LeakyIntegratorAlpha': 'leaky_integrator_alpha',
                          'LeakyIntegratorExp': 'leaky_integrator_exp',
                          'NCSource': 'nc_source',
                          'Poisson': 'poisson'}
        for k in __device_types.keys():
            self.assertEqual(get_device_name(k), __device_types[k])

    def test_print_expression_property_none(self):
        ar = api.ArgumentReference()
        ar.property = None
        ar.name = 'Test'
        self.assertEqual(print_expression(ar), ar.name)

    def test_print_expression_exception(self):
        x = None
        self.assertRaises(Exception, print_expression, x)
        self.assertRaises(Exception, get_neurons_index, x)

    def test_print_neurons_index(self):
        idx = api.Index()
        idx.population = "Foo"
        idx.index = 42
        self.assertEqual(get_neurons_index(idx), str(idx.index))
        self.assertEqual(print_neurons(idx), "nrp.brain.Foo[42]")

    def test_print_neurons_range(self):
        rng = api.Range()
        rng.population = "Foo"
        rng.from_ = 7
        rng.to = 12
        self.assertEqual(get_neurons_index(rng), "slice(7, 12)")
        self.assertEqual(print_neurons(rng), "nrp.brain.Foo[slice(7, 12)]")
        rng.step = 2
        self.assertEqual(get_neurons_index(rng), "slice(7, 12, 2)")
        self.assertEqual(print_neurons(rng), "nrp.brain.Foo[slice(7, 12, 2)]")

    def test_print_neurons_list(self):
        lst = api.List()
        lst.population = "Foo"
        self.assertEqual(get_neurons_index(lst), "[]")
        self.assertEqual(print_neurons(lst), "nrp.brain.Foo[[]]")
        lst.add_element("test")
        self.assertEqual(get_neurons_index(lst), "[test]")
        self.assertEqual(print_neurons(lst), "nrp.brain.Foo[[test]]")
        lst.add_element("test2")
        self.assertEqual(get_neurons_index(lst), "[test, test2]")
        self.assertEqual(print_neurons(lst), "nrp.brain.Foo[[test, test2]]")

    def test_print_neurons_population(self):
        pop = api.Population()
        pop.population = "Foo"
        pop.count = 42
        self.assertEqual(get_neurons_index(pop), None)
        self.assertEqual(print_neurons(pop), "nrp.brain.Foo")

    def test_print_neurons_index_template(self):
        idx = api.IndexTemplate()
        idx.index = "i+3"
        self.assertEqual(get_neuron_template(idx), "[i+3]")

    def test_print_neurons_range_template(self):
        rng = api.RangeTemplate()
        rng.from_ = "i+7"
        rng.to = "2*i+12"
        self.assertEqual(get_neuron_template(rng), "[slice(i+7, 2*i+12)]")
        rng.step = 2
        self.assertEqual(get_neuron_template(rng), "[slice(i+7, 2*i+12, 2)]")

    def test_print_neurons_list_template(self):
        lst = api.ListTemplate()
        self.assertEqual(get_neuron_template(lst), "[[]]")
        lst.add_element("i")
        self.assertEqual(get_neuron_template(lst), "[[i]]")
        lst.add_element("2*(i+1)")
        self.assertEqual(get_neuron_template(lst), "[[i, 2*(i+1)]]")

    def test_print_template_raises(self):
        self.assertRaises(Exception, get_neuron_template, None)

    def test_get_collection_source_range(self):
        rng = api.Range()
        rng.population = "Foo"
        rng.from_ = 7
        rng.to = 12
        self.assertEqual(get_collection_source(rng), "range(7, 12)")
        rng.step = 2
        self.assertEqual(get_collection_source(rng), "range(7, 12, 2)")

    def test_get_collection_source_list(self):
        lst = api.List()
        lst.population = "Foo"
        self.assertEqual(get_collection_source(lst), "[]")
        lst.add_element("test")
        self.assertEqual(get_collection_source(lst), "[test]")
        lst.add_element("test2")
        self.assertEqual(get_collection_source(lst), "[test, test2]")

    def test_get_collection_source_population(self):
        pop = api.Population()
        pop.population = "Foo"
        pop.count = 42
        self.assertEqual(get_collection_source(pop), "range(0, 42)")

    def test_get_collection_source_raises(self):
        self.assertRaises(Exception, get_collection_source, None)

    def test_print_neurons_exception(self):
        x = None
        self.assertRaises(Exception, get_neurons_index, x)

    def test_print_neuron_group_chain(self):
        chain = api.ChainSelector()
        chain.add_neurons(api.Population("Foo", 42))
        self.assertEqual(print_neuron_group(chain), "nrp.chain_neurons(nrp.brain.Foo)")
        chain.add_neurons(api.Population("Bar", 23))
        self.assertEqual(print_neuron_group(chain),
                         "nrp.chain_neurons(nrp.brain.Foo, nrp.brain.Bar)")
        chain.add_connectors(api.ChainSelector())
        self.assertEqual(print_neuron_group(chain),
                         "nrp.chain_neurons(nrp.brain.Foo, nrp.brain.Bar, nrp.chain_neurons())")

    def test_print_neuron_group_map(self):
        map = api.MapSelector()
        map.source = api.Population("Foo", 42)
        map.pattern = api.IndexTemplate("i")
        self.assertEqual(print_neuron_group(map),
                         "nrp.map_neurons(range(0, 42), lambda i: nrp.brain.Foo[i])")


if __name__ == '__main__':
    unittest.main()