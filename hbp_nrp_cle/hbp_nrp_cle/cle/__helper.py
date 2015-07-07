"""
This module contains CLE helper functionality shared among SerialClosedLoopEngine and
ClosedLoopEngine implementations
"""

from hbp_nrp_cle.tf_framework import ITransferFunctionManager

__author__ = 'GeorgHinkel'


def get_tf_elapsed_times(tfm):
    """
    Gets the Transfer Functions from the given Transfer Function manager in a dictionary

    :param tfm: The Transfer Function Manager
    :return: A dictionary with the elapsed times
    """
    times = {}
    assert isinstance(tfm, ITransferFunctionManager)
    for tf in tfm.transfer_functions():
        times[tf.name] = tf.elapsed_time
    return times
