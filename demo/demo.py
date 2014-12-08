# -*- coding: utf-8 -*-
"""
CLE integration demo
"""

__author__ = 'LorenzoVannucci'

from hbp_nrp_cle.cle.SerialClosedLoopEngine import SerialClosedLoopEngine \
    as ClosedLoopEngine

import PyNNScript
from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

from hbp_nrp_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
from hbp_nrp_cle.brainsim.PyNNCommunicationAdapter import \
    PyNNCommunicationAdapter

from hbp_nrp_cle.tf_framework import config
# pylint: disable=F0401,W0611
import MS2TransferFunctions

import signal


# consts
TIMESTEP = 0.01
MAX_SIM_TIME = 20.0


# SIGINT handler
flag = False


# pylint: disable=W0603,W0613
def handler(signum, frame):
    '''
    Handler function
    '''
    global flag
    flag = True

signal.signal(signal.SIGINT, handler)


# CREATE INTERFACE TO GAZEBO
# control adapter
roscontrol = RosControlAdapter()

# communication adapter
roscomm = RosCommunicationAdapter()

# CREATE INTERFACE TO BRAIN
# control adapter
braincontrol = PyNNControlAdapter()
braincontrol.initialize()

PyNNScript.init_brain_simulation()

# communication adapter
braincomm = PyNNCommunicationAdapter()

# CREATE TRANSFER FUNCTION MANAGER
# tf manager
tfmanager = config.active_node
#assert isinstance(tfmanager, _TransferFunctionManager.TransferFunctionManager)

# set adapters
tfmanager.robot_adapter = roscomm
tfmanager.brain_adapter = braincomm

# set transfer functions
# TODO

# Create CLE
cle = ClosedLoopEngine(roscontrol, braincontrol, tfmanager, TIMESTEP)

# initialize everything
cle.initialize()

# run simulation
for i in range(0, int(MAX_SIM_TIME / TIMESTEP)):
    if flag:
        break
    cle.run_step(TIMESTEP)

cle.shutdown()
