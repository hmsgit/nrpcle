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
from mock import patch, PropertyMock, Mock, MagicMock
from hbp_nrp_cle.robotsim.RosRobotControlAdapter import RosRobotControlAdapter
import unittest
from testfixtures import LogCapture

__author__ = 'Lorenzo Vannucci'


class TestRosRobotControlAdapter(unittest.TestCase):

    def setUp(self):
        self._rca = RosRobotControlAdapter()
        self._rca.initialize()

    def test_initialize(self):

        with LogCapture('hbp_nrp_cle.robotsim.RosRobotControlAdapter') as l:
            rca = RosRobotControlAdapter()
            rca.initialize()
            self.assertEqual(rca.time_step, 0.0)

            l.check(('hbp_nrp_cle.robotsim.RosRobotControlAdapter', 'INFO',
                     'Robot control adapter initialized'))

    def test_time_step(self):
        self.assertTrue(self._rca.set_time_step(0.01))
        self.assertEquals(self._rca.time_step, 0.0)

    def test_is_paused(self):
        self.assertFalse(self._rca.is_paused)

    def test_is_alive(self):
        self.assertTrue(self._rca.is_alive)

    def test_run_step(self):
        result = self._rca.run_step(0.01)
        result = self._rca.run_step(0.01)
        self.assertAlmostEqual(result, 0.02, 3)

    def test_run_step_async(self):
        result = self._rca.run_step_async(0.01)
        self.assertTrue(result.end - result.start < 0.01)

    def test_reset(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosRobotControlAdapter') as logcapture:
            self._rca.run_step(0.05)
            self._rca.reset()
            logcapture.check(('hbp_nrp_cle.robotsim.RosRobotControlAdapter', 'INFO',
                              'Please reset the robot'))

    def test_reset_world(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosRobotControlAdapter') as logcapture:
            self._rca.reset_world(None, None)
            logcapture.check(('hbp_nrp_cle.robotsim.RosRobotControlAdapter', 'INFO',
                              'Please reset the environment around the robot'))

    def test_shutdown(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosRobotControlAdapter') as logcapture:
            self._rca.shutdown()
            logcapture.check(('hbp_nrp_cle.robotsim.RosRobotControlAdapter', 'INFO',
                              'Please shutdown the robot'))


if __name__ == '__main__':
    unittest.main()
