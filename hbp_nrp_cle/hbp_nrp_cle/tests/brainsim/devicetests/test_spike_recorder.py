from hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder import PyNNNestSpikeRecorder as SpikeRecorder
import unittest
from mock import patch, Mock

__author__ = 'Georg Hinkel'


class TestSpikeRecorder(unittest.TestCase):

    def setUp(self):
        pass

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder.nest")
    def test_default_config(self, nest_mock):
        dev = SpikeRecorder()
        neurons = Mock()
        dev.connect(neurons)
        neurons.recorder.reset.assert_called_once_with()
        neurons.record.assert_called_once_with("spikes", to_file=False)
        nest_mock.SetStatus.assert_any_call(neurons.recorder._spike_detector.device, "to_memory", True)
        nest_mock.SetStatus.assert_any_call(neurons.recorder._spike_detector.device, "to_file", False)
        self.assertDictEqual(dev._parameters, {
            'use_ids': True
        })

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder.nest")
    def test_use_indices_config(self, nest_mock):
        dev = SpikeRecorder(use_ids = False)
        neurons = Mock()
        dev.connect(neurons)
        neurons.recorder.reset.assert_called_once_with()
        neurons.record.assert_called_once_with("spikes", to_file=False)
        nest_mock.SetStatus.assert_any_call(neurons.recorder._spike_detector.device, "to_memory", True)
        nest_mock.SetStatus.assert_any_call(neurons.recorder._spike_detector.device, "to_file", False)
        self.assertDictEqual(dev._parameters, {
            'use_ids': False
        })

    @staticmethod
    def id_to_index(index):
        if index == 9:
            return 1
        elif index == 10:
            return 2
        else:
            raise IndexError()

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder.nest")
    def test_default_refresh_config(self, nest_mock):
        dev = SpikeRecorder()
        neurons = Mock()
        dev.connect(neurons)
        nest_mock.GetStatus.return_value = [{
            'times': [0, 8, 15],
            'senders': [8, 9, 10]
        }]

        neurons.id_to_index = TestSpikeRecorder.id_to_index
        dev.refresh(0.0)

        spikes = dev.times
        self.assertEqual(3, len(spikes))
        print spikes
        self.assertEqual(8.0, spikes[0][0])

    @patch("hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder.nest")
    def test_use_indices_refresh_config(self, nest_mock):
        dev = SpikeRecorder(use_ids = False)
        neurons = Mock()
        dev.connect(neurons)
        nest_mock.GetStatus.return_value = [{
            'times': [0, 8, 15],
            'senders': [8, 9, 10]
        }]

        neurons.id_to_index = TestSpikeRecorder.id_to_index
        dev.refresh(0.0)

        spikes = dev.times
        self.assertEqual(2, len(spikes))
        print spikes
        self.assertEqual(1.0, spikes[0][0])