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

from hbp_nrp_cle.brainsim.pynn_spiNNaker.PyNNSpiNNakerControlAdapter import PySpiNNakerControlAdapter
import unittest
from mock import Mock
import sys


class CellType(object):
    def __init__(self):
        self.default_parameters = ["foo"]

    def get_value(self, parameter):
        return "bar"


class Population(Mock):
    def __init__(self):
        super(Population, self).__init__()
        self._all_ids = [0, 8, 15]
        self.celltype = CellType()

setup = Mock()


class TestSpinnakerController(unittest.TestCase):
    def setUp(self):
        mod = sys.modules[__name__]
        self.sim = mod
        self.sim.setup.reset_mock()
        self.controller = PySpiNNakerControlAdapter(mod)

    def test_spinnaker_controller_init(self):
        self.controller.initialize()
        self.sim.setup.assert_called_once_with(timestep=1.0, min_delay=1.0, max_delay=20.0)
        self.sim.setup.reset_mock()
        self.controller._PyNNControlAdapter__is_initialized = False
        self.controller.initialize(timestep=0.5, min_delay=2.0)
        self.sim.setup.assert_called_once_with(timestep=0.5, min_delay=2.0, max_delay=20.0)

    def test_population_info(self):
        p = Population()
        self.assertSequenceEqual(p.all(), [0, 8, 15])
        self.assertTrue(self.controller._is_population(p))
        self.assertFalse(self.controller._is_population("foo"))
        info = self.controller._create_population_info(p, "foo")

        self.assertEqual("CellType", info.celltype)
        self.assertEqual("foo", info.name)
        self.assertDictEqual({"foo": "bar"}, info.parameters)