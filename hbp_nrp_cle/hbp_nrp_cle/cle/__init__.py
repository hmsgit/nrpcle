"""
The package contains the implementation of the Closed Loop Engine.

The CLE interface grants complete control over both the physic and the
neural simulations. The two simulation can be started, advanced, paused and
resetted in a synchronous manner.

Two implementation of the CLE interface are given:

a.  ClosedLoopEngine is a fully parallel implementation in which the physics
    simulation, the neural simulation and the transfer functions run in
    separate threads. Currently, a bug in NEST makes it impossible to use
    this implementation as running the neural simulation in a separate thread
    causes a segmentation fault.

b.  SerialClosedLoopEngine is an implementation that overcomes the NEST bug
    by running the neural simulation and the transfer functions in the same
    thread, obviously reducing the performances.

The ROSCLEServer module provides utility classes to run the Closed Loop
Engine in a separate process, while communicating with it through ROS services.
"""
from hbp_nrp_cle import config

__author__ = 'LorenzoVannucci'

ROS_CLE_NODE_NAME = config.config.get('ros', 'ros-cle-node-name')
SERVICE_VERSION = '/%s/version' % (ROS_CLE_NODE_NAME, )
TOPIC_STATUS = '/%s/status' % (ROS_CLE_NODE_NAME, )
SERVICE_START_NEW_SIMULATION = '/%s/start_new_simulation' % (ROS_CLE_NODE_NAME, )
SERVICE_SIM_START_ID = lambda sim_id: '/%s/%d/start' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STOP_ID = lambda sim_id: '/%s/%d/stop' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_PAUSE_ID = lambda sim_id: '/%s/%d/pause' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_RESET_ID = lambda sim_id: '/%s/%d/reset' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STATE_ID = lambda sim_id: '/%s/%d/state' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_TRANSFER_FUNCTIONS = lambda sim_id: \
                                 '/%s/%d/get_transfer_functions' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SET_TRANSFER_FUNCTION = lambda sim_id: \
                                 '/%s/%d/set_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_DELETE_TRANSFER_FUNCTION = lambda sim_id: \
                                 '/%s/%d/delete_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)


def ros_handler(func):
    """
    A decorator for ROS handlers.

    :param func: the function to decorate.
    :return: a wrapped function
    """

    def inner(*args, **kwargs):
        """
        A function that wraps func returning [] when func returns None.
        """
        q = func(*args, **kwargs)
        return [] if q is None else q

    return inner
