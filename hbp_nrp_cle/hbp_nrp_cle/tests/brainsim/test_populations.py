# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
