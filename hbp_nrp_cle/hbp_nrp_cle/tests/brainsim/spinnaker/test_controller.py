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

class TestSpinnakerController(unittest.TestCase):
    def test_spinnaker_controller_init(self):
        sim = Mock()
        controller = PySpiNNakerControlAdapter(sim)
        controller.initialize()
        sim.setup.assert_called_once_with(timestep=1.0, min_delay=1.0, max_delay=20.0)
        sim.setup.reset_mock()
        controller._PyNNControlAdapter__is_initialized = False
        controller.initialize(timestep=0.5, min_delay=2.0)
        sim.setup.assert_called_once_with(timestep=0.5, min_delay=2.0, max_delay=20.0)