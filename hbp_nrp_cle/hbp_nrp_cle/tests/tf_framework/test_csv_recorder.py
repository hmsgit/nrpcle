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
        mocked_csv_writer.writerows.assert_called_with([['header1', 'header2'], (1,2)])

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
