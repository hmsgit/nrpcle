from hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestACSource import PyNNNestACSource
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


@patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup.nest")
class TestNestDeviceGroup(unittest.TestCase):

    def __create_mock_with_id(self, i):
        m = Mock()
        m._device = [i]
        return m

    @patch("hbp_nrp_cle.brainsim.pynn.devices.__PyNNACSource.sim")
    def setUp(self, mocked_sim):
        self.neurons = [self.__create_mock_with_id(i) for i in range(0,5)]
        mocked_sim.ACSource.side_effect = self.neurons
        self.device = PyNNNestACSource.create_new_device_group(len(self.neurons), {})
        self.device_ids = self.device._device_ids

        self.assertEqual(len(self.neurons), len(self.device_ids))
        for i in range(0,5):
            self.assertEqual(i, self.device_ids[i])

    def test_device_group_get_one_nest_get_status(self, mocked_nest):
        mocked_nest.GetStatus.return_value = 42
        frequency = self.device.frequency
        mocked_nest.GetStatus.assert_called_once_with(self.device_ids, "frequency")
        self.assertEqual(42, frequency)

    def test_device_group_get_transform_one_nest_get_status(self, mocked_nest):
        mocked_nest.GetStatus.return_value = [42000, 0, 8000, 15000]
        amplitude = self.device.amplitude
        mocked_nest.GetStatus.assert_called_once_with(self.device_ids, "amplitude")
        self.assertItemsEqual([42.0, 0.0, 8.0, 15.0], amplitude)

    def test_device_group_single_set_one_nest_set_status(self, mocked_nest):
        val = 42
        self.device.frequency = val
        self.assertEqual(1, mocked_nest.SetStatus.call_count)
        call_args = mocked_nest.SetStatus.call_args[0]
        self.assertEqual(self.device_ids, call_args[0])
        self.assertDictEqual({"frequency": 42}, call_args[1])

    def test_device_group_single_set_transform_one_nest_set_status(self, mocked_nest):
        val = 42
        self.device.amplitude = val
        self.assertEqual(1, mocked_nest.SetStatus.call_count)
        call_args = mocked_nest.SetStatus.call_args[0]
        self.assertEqual(self.device_ids, call_args[0])
        self.assertDictEqual({"amplitude": 42000.0}, call_args[1])

    def test_device_group_multiple_set_one_nest_set_status(self, mocked_nest):
        val = [42, 0, 8, 15, 0]
        self.device.frequency = val
        self.assertEqual(1, mocked_nest.SetStatus.call_count)
        call_args = mocked_nest.SetStatus.call_args[0]
        self.assertEqual(self.device_ids, call_args[0])
        self.assertEqual(5, len(call_args[1]))
        for i in range(0,5):
            self.assertDictEqual({"frequency": val[i]}, call_args[1][i])

    def test_device_group_multiple_set_transform_one_nest_set_status(self, mocked_nest):
        val = [42, 0, 8, 15, 0]
        self.device.amplitude = val
        self.assertEqual(1, mocked_nest.SetStatus.call_count)
        call_args = mocked_nest.SetStatus.call_args[0]
        self.assertEqual(self.device_ids, call_args[0])
        self.assertEqual(5, len(call_args[1]))
        for i in range(0,5):
            self.assertDictEqual({"amplitude": val[i] * 1000.0}, call_args[1][i])