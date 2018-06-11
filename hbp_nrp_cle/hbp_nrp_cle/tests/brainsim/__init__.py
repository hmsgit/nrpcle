import os

NRP_SIMULATION_DIR = None


def setUpModule():
    global NRP_SIMULATION_DIR
    NRP_SIMULATION_DIR = os.environ.get('NRP_SIMULATION_DIR')
    if NRP_SIMULATION_DIR is not None:
        del os.environ['NRP_SIMULATION_DIR']


def tearDownModule():
    if NRP_SIMULATION_DIR is not None:
        os.environ['NRP_SIMULATION_DIR'] = NRP_SIMULATION_DIR