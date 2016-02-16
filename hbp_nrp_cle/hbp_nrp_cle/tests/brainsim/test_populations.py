"""
This tests the ability of the PyNNControlAdapter to correctly load neuron populations
"""

__author__ = 'Georg Hinkel'

import unittest
import os
from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter

class TestPyNNControlAdapter(unittest.TestCase):

    def test_populations(self):
        adapter = PyNNControlAdapter()
        directory = os.path.dirname(__file__)
        adapter.load_brain(os.path.join(directory, "example_pynn_brain.py"))
        populations = adapter.get_populations()
        self.assertEqual(3, len(populations))
        # Populations may be in any order, so we have to crawl the collection
        population = next(p for p in populations if p.name == "population")
        view = next(p for p in populations if p.name == "view")
        assembly = next(p for p in populations if p.name == "assembly")
        # check population
        self.assertEqual("population", population.name)
        self.assertEqual("IF_curr_alpha", population.celltype)
        self.assertNotEqual(0, len(population.parameters))
        self.assertEqual(3, len(population.gids))
        # check view
        self.assertEqual("view", view.name)
        self.assertEqual("IF_curr_alpha", view.celltype)
        self.assertNotEqual(0, len(view.parameters))
        self.assertEqual(1, len(view.gids))
        # check assembly
        self.assertEqual("assembly", assembly.name)
        self.assertEqual("PopulationAssembly", assembly.celltype)
        self.assertEqual(0, len(assembly.parameters))
        self.assertEqual(3, len(assembly.gids))
