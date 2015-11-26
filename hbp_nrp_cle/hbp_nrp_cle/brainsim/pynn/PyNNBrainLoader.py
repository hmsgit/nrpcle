"""
Load a brain network
"""

__author__ = "Lorenzo Vannucci"

import pyNN.nest as sim
import numpy as np
import imp
from progressbar import ProgressBar, Percentage, Bar, ETA
import logging

logger = logging.getLogger("BrainLoader")
__brainIndex = 0


# pylint: disable=R0914
# the variables are reasonable in this case
def load_pointneuron_circuit(h5_filename, neuron_ids=None,
                             synapse_model='TsodyksMarkramMechanism'):
    """Loads the h5 point-neuron circuit into PyNN. The result dictionary will
    contain a set of PyNN neurons that are correctly connected together.

    .. WARNING::
        The output NEST neurons will be of type 'aeif_cond_exp' and will have
        the default NEST parameters. The dictionary that is returned by this
        function will contains a set of better parameter that can be used
        (mtype, a, b, V_th, Delta_T, C_m, g_L, V_reset, tau_w, t_ref, V_peak,
        E_L, E_ex, E_in, excitatory).

    .. WARNING::
        If the input file has been collapsed using
        'convert_h5_to_collapsed_h5', collapsed
        need to be set to True. The synapse model will then be forced to
        'quantal_stp_synapse'.

    .. WARNING::
        This function will not reset NEST kernel. GIDs of NEST neurons
        and bluepy neurons might be different.

    :param str h5_filename: Name of the h5 datafile.
    :param neuron_ids: The list of neurons ids for which connection will be
        done (inbetween them and also from them to the others. This can be
        feed for example with the results of the :func:`get_target`
        function from bluepy.)
    :type neuron_ids: list of ids
    :param str synapse_model: Specifies the synapse model that will be used.
    """
    import h5py

    logger.info("Opening h5 datafile \"" + h5_filename + "\" ... ")
    h5file = h5py.File(h5_filename, 'r')
    slist = range(1, len(h5file["x"].value) + 1)
    if neuron_ids is not None:
        slist = neuron_ids
    id_max = max(slist)
    cells = sim.Population(id_max, sim.EIF_cond_alpha_isfa_ista)  # AdEx

    pbar = ProgressBar(widgets=['Loading model',
                                Percentage(),
                                Bar(),
                                ETA()],
                       maxval=len(slist)).start()

    if synapse_model == 'TsodyksMarkramMechanism':
        tso_model = sim.TsodyksMarkramMechanism()
        syndynamics = sim.SynapseDynamics(fast=tso_model)

    i = 0
    for i, gid_ in enumerate(slist):

        pbar.update(i)

        if "syn_" + str(gid_) in h5file:
            r_syns = h5file["syn_" + str(gid_)].value
            ids_that_are_in = np.nonzero(np.in1d(r_syns[0], slist))[0]

            # The default is to connect it using 5 parameters given below with
            # the method provided. However if the method provided has different
            # parameters you need to add a new elif a few lines below with the
            # synapse specific parameters
            synapse_parameters = \
                {"weight": np.float64(r_syns[2][ids_that_are_in]) * 1e-3,
                 "delay": np.float64(r_syns[1][ids_that_are_in]),
                 'U': np.float64(r_syns[3][ids_that_are_in]),
                 'tau_rec': np.float64(r_syns[4][ids_that_are_in]),
                 'tau_facil': np.float64(r_syns[5][ids_that_are_in]),
                 "target": np.float64(r_syns[0][ids_that_are_in])}

            if len(ids_that_are_in) == 1:
                pre = i
                post = synapse_parameters["target"] - 1
                conn_list = np.array([[pre, post,
                                       np.abs(synapse_parameters["weight"]),
                                       synapse_parameters["delay"]]])
            else:
                pre = np.ones(len(synapse_parameters["target"])) * i
                post = synapse_parameters["target"] - 1
                conn_list = np.array([pre, post,
                                      np.abs(synapse_parameters["weight"]),
                                      synapse_parameters["delay"]]).T
            connector = sim.FromListConnector(conn_list)
            if h5file["excitatory"][i] > 100:
                proj = sim.Projection(cells, cells, method=connector,
                                      synapse_dynamics=syndynamics,
                                      target='excitatory')
            else:
                proj = sim.Projection(cells, cells, method=connector,
                                      synapse_dynamics=syndynamics,
                                      target='inhibitory')
            proj.setSynapseDynamics('U', synapse_parameters['U'])
            proj.setSynapseDynamics('tau_rec', synapse_parameters['tau_rec'])
            proj.setSynapseDynamics('tau_facil',
                                    synapse_parameters['tau_facil'])

    circuit = {
        "population": cells,
        "x": np.float64(h5file["x"].value[np.array(slist) - 1]),
        "y": np.float64(h5file["y"].value[np.array(slist) - 1]),
        "z": np.float64(h5file["z"].value[np.array(slist) - 1]),
        "layer": np.int16(h5file["layer"].value[np.array(slist) - 1]),
        "mtype": h5file["mtype"].value[np.array(slist) - 1],
        "a": np.float64(h5file["a"].value[np.array(slist) - 1]),
        "b": np.float64(h5file["b"].value[np.array(slist) - 1]),
        "V_th": np.float64(h5file["V_th"].value[np.array(slist) - 1]),
        "Delta_T": np.float64(h5file["Delta_T"].value[np.array(slist) - 1]),
        "C_m": np.float64(h5file["C_m"].value[np.array(slist) - 1]),
        "g_L": np.float64(h5file["g_L"].value[np.array(slist) - 1]),
        "V_reset": np.float64(h5file["V_reset"].value[np.array(slist) - 1]),
        "tau_w": np.float64(h5file["tau_w"].value[np.array(slist) - 1]),
        "t_ref": np.float64(h5file["t_ref"].value[np.array(slist) - 1]),
        "V_peak": np.float64(h5file["V_peak"].value[np.array(slist) - 1]),
        "E_L": np.float64(h5file["E_L"].value[np.array(slist) - 1]),
        "E_ex": np.float64(h5file["E_ex"].value[np.array(slist) - 1]),
        "E_in": np.float64(h5file["E_in"].value[np.array(slist) - 1]),
        "excitatory": h5file["excitatory"].value[np.array(slist) - 1]
    }

    # This is temporary. While the synaptic decay time constants are given
    # for each synapse in the h5 file, NEST can only set 2 time constants
    # per neuron in a simulation. Thus, the averages of the synaptic values
    # had to be taken for both excitatory and inhibitory neurons. As the
    # previous default NEST values where tau_syn_ex=0.2ms and tau_syn_in=2.0ms,
    # this scales up the weights, as the total amount of charges flowing
    # through a synapse into the neuron is proportional to this time constant.
    if "tau_syn_E" in h5file:
        circuit["tau_syn_E"] = \
            np.float64(h5file["tau_syn_E"].value[np.array(slist) - 1])
    else:
        circuit["tau_syn_E"] = np.array([1.8 for i in slist], dtype=np.float64)
    if "tau_syn_I" in h5file:
        circuit["tau_syn_I"] = \
            np.float64(h5file["tau_syn_I"].value[np.array(slist) - 1])
    else:
        circuit["tau_syn_I"] = np.array([8.0 for i in slist], dtype=np.float64)

    return circuit


def load_h5_network(path, populations):
    """
    Load a h5 brain network file.

    :param path: path to the .h5 file.
    :param populations: A dictionary of the populations and their ids
    """
    # Load point neuron circuit
    circuit = load_pointneuron_circuit(path)
    population = circuit['population']

    # AdEx parameters are set
    population.tset('a', circuit["a"])        # nS
    population.tset('b', circuit["b"] * 1e-3)        # pA -> nA
    population.tset('v_thresh', circuit["V_th"])
    population.tset('delta_T', circuit["Delta_T"])
    population.tset('cm', circuit["C_m"] * 1e-3)     # pF -> nF
    population.tset('tau_m', circuit["C_m"] / circuit["g_L"])
    population.tset('v_reset', circuit["V_reset"])
    population.tset('tau_w', circuit["tau_w"])
    population.tset('tau_refrac', circuit["t_ref"])
    population.tset('v_spike', circuit["V_peak"])
    population.tset('v_rest', circuit["E_L"])
    population.tset('e_rev_E', circuit["E_ex"])
    population.tset('e_rev_I', circuit["E_in"])
    population.tset('tau_syn_E', circuit["tau_syn_E"])
    population.tset('tau_syn_I', circuit["tau_syn_I"])

    sim.initialize(population, 'v',
                   population.get('v_rest'))

    # set sensors and actors
    brain = Brain(population)

    for p in populations:
        neurons = sim.PopulationView(population, populations[p])
        brain.__dict__[p] = neurons

    import hbp_nrp_cle.tf_framework.config as tf_config
    tf_config.brain_root = brain


# pylint: disable=W0603
def load_py_network(path, populations):
    """
    Load a python network file

    :param path: path to the .py file
    :param populations: A dictionary of the populations and their ids
    """
    logger.info("Loading brain model from python: " + path)
    global __brainIndex
    brain_module = imp.load_source('__brain_model' + str(__brainIndex), path)
    __brainIndex += 1
    try:
        circuit = brain_module.circuit
        logger.info("Found circuit")
        for p in populations:
            neurons = sim.PopulationView(circuit, populations[p])
            brain_module.__dict__[p] = neurons
    except AttributeError:
        if len(populations) > 0:
            raise Exception("Could not initialize populations, no circuit found")

    import hbp_nrp_cle.tf_framework.config as tf_config
    tf_config.brain_root = brain_module


class Brain(object):
    """
    Represents a simple model of a generic brain.
    """
    def __init__(self, circuit):
        self.__circuit = circuit

    @property
    def circuit(self):
        """
        Gets the circuit brain region
        """
        return self.__circuit
