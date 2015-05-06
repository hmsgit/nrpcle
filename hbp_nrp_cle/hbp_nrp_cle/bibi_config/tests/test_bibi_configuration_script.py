"""
Test to run the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.bibi_config.bibi_configuration_script import *
import unittest
import difflib
import os


class TestBibiConfigurationScript(unittest.TestCase):

    def test_remove_extension(self):
        self.assertEqual(remove_extension("abracadab.ra"), "abracadab")
        self.assertEqual(remove_extension("file.ext"), "file")

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
        ar = generated_bibi_api.ArgumentReference()
        ar.property = None
        ar.name = 'Test'
        self.assertEqual(print_expression(ar), ar.name)

    def test_print_expression_excpeption(self):
        x = None
        self.assertRaises(Exception, print_expression, x)

    def test_print_neurons_index(self):
        idx = generated_bibi_api.Index()
        self.assertEqual(print_neurons(idx), str(idx.index))

    def test_print_neurons_list(self):
        lst = generated_bibi_api.List()
        self.assertEqual(print_neurons(lst), "[]")
        lst.add_element("test")
        self.assertEqual(print_neurons(lst), "[test]")
        lst.add_element("test")
        self.assertEqual(print_neurons(lst), "[test, test]")

    def test_print_neurons_exception(self):
        x = None
        self.assertRaises(Exception, print_neurons, x)

    def test_script(self):
        directory = os.path.split(__file__)[0]
        milestone2 = os.path.join(directory, 'milestone2.xml')
        generated_script = os.path.join(directory, 'generated_cle_script.py')
        milestone2_script = os.path.join(directory, 'milestone2_cle_script.py')
        generate_cle(milestone2, generated_script, 300.0, 'local')

        file1 = open(generated_script, 'r')
        file2 = open(milestone2_script, 'r')
        diff = difflib.context_diff(file1.readlines(), file2.readlines())
        delta = ''.join(diff)
        self.maxDiff = None
        self.assertMultiLineEqual(delta, "")


if __name__ == '__main__':
    unittest.main()
