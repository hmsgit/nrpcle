"""
Test to run the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.bibi_config.bibi_configuration_script import generate_cle
import unittest
import difflib
import os


class TestBibiConfigurationScript(unittest.TestCase):

    def test_script(self):
        directory = os.path.split(__file__)[0]
        milestone2 = os.path.join(directory, 'milestone2_python_tfs.xml')
        generated_script = os.path.join(directory, 'generated_cle_script.py')
        milestone2_script = os.path.join(directory, 'milestone2_python_tfs_cle_script.py')
        generate_cle(milestone2, generated_script, 300.0, 'local', 0,
                     os.path.dirname(os.path.realpath(__file__)))

        file1 = open(generated_script, 'r')
        file2 = open(milestone2_script, 'r')
        diff = difflib.context_diff(file1.readlines(), file2.readlines())
        delta = ''.join(diff)
        self.maxDiff = None
        self.assertMultiLineEqual(delta, "")


if __name__ == '__main__':
    unittest.main()
