"""
The PyNN script to create the braitenberg network as a placeholder
for more complex circuits.
"""

__author__ = 'DimitriProbst'

import pyNN.nest as sim
import python_cle.tf_framework.config as config


def init_brain_simulation():
    """
    Initializes PyNN with the neuronal network that has to be simulated
    """

    SENSORPARAMS = {'cm': 0.025,
                    'v_rest': -60.5,
                    'tau_m': 10.0,
                    'i_offset': 0.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    GO_ON_PARAMS = {'cm': 0.025,
                    'v_rest': -60.5,
                    'tau_m': 10.0,
                    'i_offset': 0.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -61.6,
                    'v_thresh': -60.51,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    # 3 sensor neurons
    SENSORS = sim.Population(3, sim.IF_cond_exp, cellparams=SENSORPARAMS)
    # Go-on-node and 2 actor neurons
    GO_ON = sim.Population(1, sim.IF_cond_exp, cellparams=GO_ON_PARAMS)
    ACTORS = sim.Population(2, sim.IF_cond_exp, cellparams=SENSORPARAMS)

    sim.initialize(SENSORS, 'v', SENSORS[0].v_rest)
    sim.initialize(GO_ON, 'v', SENSORS[0].v_rest)
    sim.initialize(ACTORS, 'v', ACTORS[0].v_rest)

    CIRCUIT = SENSORS + GO_ON + ACTORS  # Assembly of 6 neurons

    # Synaptic weights
    WEIGHT_RED_TO_ACTOR = 1.5e-4
    WEIGHT_RED_TO_GO_ON = -1.2e-3  # -2e-3
    WEIGHT_GREEN_BLUE_TO_ACTOR = 1.05e-4
    WEIGHT_GO_ON_TO_RIGHT_ACTOR = 1.4e-4
    DELAY = 0.1

    # Connect neurons
    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_RED_TO_ACTOR),
                                 delays=DELAY)
    sim.Projection(CIRCUIT[0:1], CIRCUIT[5:6], CONN, target='excitatory')
    sim.Projection(CIRCUIT[1:2], CIRCUIT[4:5], CONN, target='excitatory')
    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_RED_TO_GO_ON),
                                 delays=DELAY)
    sim.Projection(CIRCUIT[0:2], CIRCUIT[3:4], CONN, target='inhibitory')
    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_GREEN_BLUE_TO_ACTOR),
                                 delays=DELAY)
    sim.Projection(CIRCUIT[2:3], CIRCUIT[5:6], CONN, target='excitatory')
    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_GO_ON_TO_RIGHT_ACTOR),
                                 delays=DELAY)
    sim.Projection(CIRCUIT[3:4], CIRCUIT[5:6], CONN, target='excitatory')

    print "Circuit description: "
    print CIRCUIT.describe()

    config.brain_root = HuskyBrain(SENSORS, ACTORS, CIRCUIT)


class HuskyBrain(object):
    """
    Represents a simple model of the husky brain
    """
    def __init__(self, sensors, actors, circuit):
        self.__sensors = sensors
        self.__actors = actors
        self.__circuit = circuit

    @property
    def sensors(self):
        """
        Gets the sensors brain region
        """
        return self.__sensors

    @property
    def actors(self):
        """
        Gets the actors brain region
        """
        return self.__actors

    @property
    def circuit(self):
        """
        Gets the circuit brain region
        """
        return self.__circuit
