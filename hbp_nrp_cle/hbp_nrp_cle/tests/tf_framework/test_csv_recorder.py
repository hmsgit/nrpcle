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
"""
Tests for the CSVRecorder module
"""

__author__ = ''

import unittest
import tempfile
import hbp_nrp_cle
import hbp_nrp_cle.tf_framework as nrp
from mock import Mock, patch


class TestMapCSVRecorder(unittest.TestCase):

    def test_mapping(self):
        mapping = nrp.MapCSVRecorder("name", "filename", ['header1', 'header2'])
        recorder = mapping.create_adapter(None)
        self.assertTrue(isinstance(recorder, nrp.CSVRecorder))

class TestCSVRecorder(unittest.TestCase):
    
    def test_get_csv_recorder_name(self):
        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2'])
        self.assertEqual(recorder.get_csv_recorder_name(),"dummy_file.csv")
     
    def test_get_csv_headers(self):
        recorder = nrp.CSVRecorder("dummy.csv", ['header1', 'header2'])
        self.assertEqual(recorder.get_csv_headers(), ['header1,', 'header2,','Simulation_reset\n'])

    def test_cleanup(self):
        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2'])
        result = recorder.record_entry("tst1","tst2")
        self.assertEqual(recorder._CSVRecorder__values,[['tst1,', 'tst2\n']])
        self.assertFalse(recorder._CSVRecorder__reset_since_last_record)
        recorder.cleanup()
        self.assertEqual(recorder._CSVRecorder__values,[])

    def test_reset(self):
        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2\n'],erase_on_reset=True)
        result = recorder.reset('fakeTfManager')
        self.assertEqual(recorder._CSVRecorder__filename,"dummy_file.csv")
        self.assertTrue(recorder._CSVRecorder__reset_since_last_record)
        self.assertEqual(recorder._CSVRecorder__headers,['header1,', 'header2\n,', 'Simulation_reset\n'])