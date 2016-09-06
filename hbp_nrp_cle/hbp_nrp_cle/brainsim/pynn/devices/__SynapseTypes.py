"""
This module contains shared helper functions to parse synapse types

This module does not import the simulator module itself, but rather takes the module implementation
as a parameter. The reason for this is that this allows to make it transparent for unit tests
whether some of the parsing functionality has been extracted here.
"""

__author__ = 'Georg Hinkel'


def set_synapse_type(device_parameters, sim):
    """
    Corrects the synapse type for the given device parameters, relative to the given simulator
    implementation

    :param device_parameters: The device parameters of the given device
    :param sim: The simulator module
    """
    weights = device_parameters["weight"]
    delays = device_parameters["delay"]
    if not device_parameters["synapse_type"]:
        device_parameters["synapse_type"] = sim.StaticSynapse(weight=weights, delay=delays)
    elif isinstance(device_parameters["synapse_type"], dict):
        dyn = device_parameters["synapse_type"]
        try:
            if dyn["type"] == "TsodyksMarkram":
                device_parameters["synapse_type"] = \
                    sim.TsodyksMarkramSynapse(U=dyn["U"], tau_rec=dyn["tau_rec"],
                                              tau_facil=dyn["tau_facil"],
                                              weight=weights, delay=delays)
            else:
                raise Exception("Unknown synapse type {0}".format(dyn["type"]))
        except KeyError as e:
            raise Exception("The synapse definition {0} is missing the required field {1}"
                            .format(device_parameters["synapse_type"], str(e)))
