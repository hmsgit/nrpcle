"""
This is a dummy for a broken experiment script used for testing
"""

__author__ = 'georghinkel'

import hbp_nrp_cle.tests.cle.dummy_experiment_validation as val
from mock import Mock

def cle_function_init(environment_file):
    val.experiment_cle_init_called = True
    raise Exception("I just wanted to freak out!")

def shutdown(cle_server, models_path, gzweb, gzserver):
    val.experiment_shutdown_called = True