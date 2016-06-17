"""
Defines a neuron monitor
"""

import logging
from ._TransferFunction import TransferFunction
from hbp_nrp_cle.tf_framework import config
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder, ILeakyIntegratorAlpha, \
    ILeakyIntegratorExp, IPopulationRate
from hbp_nrp_cle.robotsim.RobotInterface import Topic
from cle_ros_msgs.msg import SpikeRate, SpikeEvent, SpikeData

logger = logging.getLogger(__name__)

__author__ = 'Georg Hinkel'

SPIKE_RECORDER_TOPIC = "/monitor/spike_recorder"
LEAKY_INTEGRATOR_ALPHA_TOPIC = "/monitor/leaky_integrator_alpha"
LEAKY_INTEGRATOR_EXP_TOPIC = "/monitor/leaky_integrator_exp"
POPULATION_RATE_TOPIC = "/monitor/population_rate"


class NeuronMonitor(TransferFunction):
    """
    Class to represent transfer functions used for monitoring neurons
    """

    def __init__(self, neurons, monitor_type):
        """
        Defines a new transfer function from robots to neurons

        :param neurons: The neurons that should be monitored
        :param monitor_type: The type of monitor that should be injected
        """
        super(NeuronMonitor, self).__init__()
        self.__neurons = None
        self.__device = None
        self.__type = monitor_type
        self.__publisher = None
        self.__count = 0
        self.__neurons_spec = neurons
        if monitor_type is ISpikeRecorder:
            self.__handler = self.__send_spike_recorder
        elif monitor_type is ILeakyIntegratorExp or type is ILeakyIntegratorAlpha:
            self.__handler = self.__send_leaky_integrator
        elif monitor_type is IPopulationRate:
            self.__handler = self.__send_population_rate
        else:
            raise Exception("Type {0} is not a supported monitor type".format(type))

    def __call__(self, func):  # -> Neuron2Robot:
        """
        Applies the transfer functions object to the given function

        :param func: The function body for this transfer function
        :return The transfer function object
        """
        self._init_function(func, config.active_node.n2r)
        return self

    def __repr__(self):  # pragma: no cover
        return "{0} monitors {1}" \
            .format(self.name, self.__neurons_spec)

    def initialize(self, tfm, bca_changed, rca_changed):
        """
        Initializes this transfer function to be used with the given TFM

        :param tfm: The Transfer Function Manager
        :param bca_changed: True, if the brain communication adapter has changed
        :param rca_changed: True, if the robot communication adapter has changed
        """
        if bca_changed:
            self.__neurons = self.__neurons_spec.select(config.brain_root)
            self.__count = self.__neurons.size
            self.__device = tfm.brain_adapter.register_spike_sink(self.__neurons, self.__type)
        if rca_changed:
            _topic = None
            _type = SpikeRate
            if self.__type is ISpikeRecorder:
                _topic = SPIKE_RECORDER_TOPIC
                _type = SpikeEvent
            elif self.__type is ILeakyIntegratorAlpha:
                _topic = LEAKY_INTEGRATOR_ALPHA_TOPIC
            elif self.__type is ILeakyIntegratorExp:
                _topic = LEAKY_INTEGRATOR_EXP_TOPIC
            elif self.__type is IPopulationRate:
                _topic = POPULATION_RATE_TOPIC
            self.__publisher = tfm.robot_adapter.register_publish_topic(Topic(_topic, _type))

    def __send_spike_recorder(self, t):
        """
        Sends spike data to the given spike recorder monitoring topic

        :param t: The simulation time
        :return:
        """
        spikes = self.__device.times
        msgs = []
        for spike in spikes:
            try:
                msgs.append(SpikeData(self.__neurons.id_to_index(int(spike[0])), spike[1]))
            except IndexError:
                pass
        self.__publisher.send_message(SpikeEvent(t, self.__count, msgs, self.name))

    def __send_leaky_integrator(self, t):
        """
        Sends spike data to the given leaky integrator monitoring topic

        :param t: The simulation time
        :return:
        """
        self.__publisher.send_message(SpikeRate(t, self.__device.voltage, self.name))

    def __send_population_rate(self, t):
        """
        Sends spike data to the given population rate monitoring topic

        :param t: The simulation time
        :return:
        """
        self.__publisher.send_message(SpikeRate(t, self.__device.rate, self.name))

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time

        :param t: The simulation time
        """

        return_value = super(NeuronMonitor, self).run(t)

        if return_value:
            self.__handler(t)
