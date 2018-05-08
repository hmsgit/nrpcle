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

    def test_record_entry_needs_the_correct_number_of_argument(self):
        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2'])
        self.assertRaises(ValueError, recorder.record_entry, 'a')
        self.assertRaises(ValueError, recorder.record_entry, 'a', 'b', 'c')
        self.assertRaises(ValueError, recorder.record_entry)
        self.assertRaises(ValueError, recorder.record_entry,['a', 'b'])

    @patch('csv.writer')
    @patch('tempfile.NamedTemporaryFile')
    def test_record_entry_and_dumps_to_file_produce_proper_results(self, mock_tempfile, mock_csv_writer):
        mocked_tempfile = Mock()
        mocked_tempfile.name = "some_temp_file_name"
        mock_tempfile.return_value.__enter__.return_value = mocked_tempfile
        mock_tempfile.return_value.__exit__.return_value = None
        mocked_csv_writer = Mock()
        mocked_csv_writer.writerows = Mock()
        mock_csv_writer.return_value = mocked_csv_writer

        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2'])
        recorder.record_entry(1,2)
        result = recorder.dump_to_file()
        self.assertEqual(result,('dummy_file.csv', 'some_temp_file_name'))
        mocked_csv_writer.writerows.assert_called_with([['header1', 'header2', 'Simulation_reset'], [1,2, '']])

    @patch('os.remove')
    @patch('csv.writer')
    @patch('tempfile.NamedTemporaryFile')
    def test_cleanup(self, mock_tempfile, mock_csv_writer, mock_os_remove):
        temp_dir = tempfile.gettempdir()
        mocked_tempfile = Mock()
        mock_tempfile.return_value.__enter__.return_value = mocked_tempfile
        mock_tempfile.return_value.__exit__.return_value = None
        mocked_csv_writer = Mock()
        mocked_csv_writer.writerows = Mock()

        mocked_tempfile.name = temp_dir + "/../etc/password-this-is-for-test"
        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2'])
        result = recorder.dump_to_file()
        recorder.cleanup()
        self.assertEqual(mock_os_remove.call_count, 0)

        mocked_tempfile.name = temp_dir + "/aaa"
        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2'])
        result = recorder.dump_to_file()
        recorder.cleanup()
        mock_os_remove.assert_called_with(temp_dir + "/aaa")

    @patch('csv.writer')
    @patch('tempfile.NamedTemporaryFile')
    def test_reset(self, mock_tempfile, mock_csv_writer):
        mocked_tempfile = Mock()
        mocked_tempfile.name = "some_temp_file_name"
        mock_tempfile.return_value.__enter__.return_value = mocked_tempfile
        mock_tempfile.return_value.__exit__.return_value = None
        mocked_csv_writer = Mock()
        mocked_csv_writer.writerows = Mock()
        mock_csv_writer.return_value = mocked_csv_writer

        recorder = nrp.CSVRecorder("dummy_file.csv", ['header1', 'header2','header3'])
        recorder.record_entry(1, 2, 3)
        reset_recorder = recorder.reset(None)
        recorder.record_entry(3, 4, 5)
        recorder.dump_to_file()

        self.assertEqual(recorder, reset_recorder)
        mocked_csv_writer.writerows.assert_called_with([['header1', 'header2', 'header3', 'Simulation_reset'],
                                                        [1,2, 3,''],
                                                        [3, 4, 5, 'RESET']])