#!/usr/bin/env python

import rospy
import smach
from smach import CBState
import smach_ros

from std_msgs.msg import String


def condition_30_cb(user_data, status):
    # ... parse JSON in status message ...
    if status.simulationTime < 30:
        return True  # condition not met so far, evaluate state outcome to still 'valid'
    else:
        return False  # condition met, evaluates state outcome to 'invalid'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_30_cb(user_data):
    # ... do something
    return 'finished'


def condition_spike_cb(user_data, spike):
    # ... parse JSON in spike message ...
    if spike < 10:
        return True  # condition not met so far, evaluate state outcome to still 'valid'
    else:
        return False  # condition met, evaluates state outcome to 'invalid'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_spike_cb(user_data):
    # ... do something
    return 'finished'

if __name__ == "__main__":
    # Create a ROS node for this state machine
    rospy.init_node("hbp_nrp_backend_sm_exp_control")

    sm_one = smach.StateMachine(
    outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
              'ACTION_PREEMPTED', 'ACTION_ERROR'])

    with sm_one:
        smach.StateMachine.add('CONDITION_30',
                               smach_ros.MonitorState('/ros_cle_simulation/status',
                                                      String,
                                                      condition_30_cb),
                               {'valid': 'CONDITION_30', 'invalid': 'ACTION_30',
                                'preempted': 'CONDITION_PREEMPTED'})
        smach.StateMachine.add('ACTION_30', CBState(action_30_cb),
                               {'finished': 'FINISHED'})

    sm_two = smach.StateMachine(
    outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
              'ACTION_PREEMPTED', 'ACTION_ERROR'])

    with sm_two:
        smach.StateMachine.add('CONDITION_SPIKE',
                               smach_ros.MonitorState('/monitoring/left_wheel_neuron_rate_monitor',
                                                      String,
                                                      condition_spike_cb),
                               {'valid': 'CONDITION_SPIKE', 'invalid': 'ACTION_SPIKE',
                                'preempted': 'CONDITION_PREEMPTED'})
        smach.StateMachine.add('ACTION_SPIKE', CBState(action_spike_cb),
                               {'finished': 'FINISHED'})

    sm = smach.Concurrence(outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED'],
                           default_outcome='ERROR', input_keys=[], output_keys=[],
                           outcome_map = {'FINISHED':{'SM_ONE':'FINISHED', 'SM_TWO':'FINISHED'},
                                     'CONDITION_PREEMPTED':{'SM_ONE':'CONDITION_PREEMPTED'},
                                     'CONDITION_PREEMPTED':{'SM_TWO':'CONDITION_PREEMPTED'}})

    with sm:
        smach.Concurrence.add('SM_ONE', sm_one)
        smach.Concurrence.add('SM_TWO', sm_two)

    result = sm.execute()
    rospy.loginfo(result)
