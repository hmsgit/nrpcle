"""
Test for single functions of the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.bibi_config.bibi_configuration_script import *
import hbp_nrp_cle.bibi_config.generated.bibi_api_gen as api
import unittest
import pyxb


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
        monitor = api.Neuron2Monitor(name="testMonitor",
                                     device=[api.DeviceChannel(type="PopulationRate", name="neurons",
                                                              neurons=api.List(population="pop", element=[1, 2, 3]))])
        self.assertEqual(get_monitoring_type(monitor), "cle_ros_msgs.msg.SpikeRate")
        self.assertEqual(get_monitoring_topic(monitor), "/monitor/population_rate")

        impl = 'cle_ros_msgs.msg.SpikeRate(t, neurons.rate, ' \
               '"testMonitor")'
        self.assertEqual(get_monitoring_impl(monitor), impl)

    def test_monitoring_spike_detector(self):
        neurons=api.List(population="pop", element=[1, 2, 3])
        dc=api.DeviceChannel(type="SpikeRecorder", name="neurons", neurons=neurons)
        monitor = api.Neuron2Monitor(name="testMonitor", device=[dc])

        self.assertEqual(get_monitoring_type(monitor), "cle_ros_msgs.msg.SpikeEvent")
        self.assertEqual(get_monitoring_topic(monitor), "/monitor/spike_recorder")

        impl = 'monitoring.create_spike_recorder_message(t, 3, neurons.times, "testMonitor")'
        self.assertEqual(get_monitoring_impl(monitor), impl)

    def test_get_neuron_count(self):
        self.assertEqual(3, get_neuron_count(api.List(population="pop", element=[1, 2, 3])))
        self.assertEqual(1, get_neuron_count(api.Index(population="pop")))
        self.assertEqual(4, get_neuron_count(api.Range(population="pop", from_=3, to=7)))
        self.assertEqual(2, get_neuron_count(api.Range(population="pop", from_=0, to=5, step=2)))
        self.assertEqual(42, get_neuron_count(api.Population(population="pop", count=42)))

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
        ar.property_ = None
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
        lst.element.append(5)
        self.assertEqual(get_neurons_index(lst), "[5]")
        self.assertEqual(print_neurons(lst), "nrp.brain.Foo[[5]]")
        lst.element.append(10)
        self.assertEqual(get_neurons_index(lst), "[5, 10]")
        self.assertEqual(print_neurons(lst), "nrp.brain.Foo[[5, 10]]")

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
        lst.element.append("i")
        self.assertEqual(get_neuron_template(lst), "[[i]]")
        lst.element.append("2*(i+1)")
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
        lst.element.append(5)
        self.assertEqual(get_collection_source(lst), "[5]")
        lst.element.append(11)
        self.assertEqual(get_collection_source(lst), "[5, 11]")

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
        chain.neurons.append(api.Population(population="Foo", count=42))
        self.assertEqual(print_neuron_group(chain), "nrp.chain_neurons(nrp.brain.Foo)")
        chain.neurons.append(api.Population(population="Bar", count=23))
        self.assertEqual(print_neuron_group(chain),
                         "nrp.chain_neurons(nrp.brain.Foo, nrp.brain.Bar)")
        chain.connectors.append(api.ChainSelector())
        self.assertEqual(print_neuron_group(chain),
                         "nrp.chain_neurons(nrp.brain.Foo, nrp.brain.Bar, nrp.chain_neurons())")

    def test_print_neuron_group_map(self):
        map = api.MapSelector()
        map.source = api.Population(population="Foo", count=42)
        map.pattern = api.IndexTemplate(index="i")
        self.assertEqual(print_neuron_group(map),
                         "nrp.map_neurons(range(0, 42), lambda i: nrp.brain.Foo[i])")

    def test_print_synapse_dynamic(self):
        syn = api.TsodyksMarkramMechanism(u=1.0, tau_rec=0.0, tau_facil=0.0)
        self.assertEqual(print_synapse_dynamics(syn),
                         "sim.SynapseDynamics(fast=sim.TsodyksMarkramMechanism(U=1.0, tau_rec=0.0, tau_facil=0.0))")
        self.assertRaises(Exception, print_synapse_dynamics, None)

    def test_print_connector(self):
        con = api.OneToOneConnector(delays=0.8, weights=42.0)
        self.assertEqual(print_connector(con),
                         "sim.OneToOneConnector(weights=42.0, delays=0.8)")
        self.assertRaises(Exception, print_connector, None)

    def test_print_device_config(self):
        dev = api.DeviceChannel()
        self.assertEqual(print_device_config(dev), "")
        dev.synapseDynamicsRef = api.SynapseDynamicsRef(ref="Foo")
        dev.connectorRef = api.NeuronConnectorRef(ref="Bar")
        dev.target = 'Inhibitory'
        self.assertEqual(print_device_config(dev), ", synapse_dynamics=Foo, connector=Bar, target='inhibitory'")

if __name__ == '__main__':
    unittest.main()