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


def condition_60_cb(user_data, status):
    # ... parse JSON in status message ...
    if status.simulationTime < 60:
        return True  # condition not met so far, evaluate state outcome to still 'valid'
    else:
        return False  # condition met, evaluates state outcome to 'invalid'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_60_cb(user_data):
    # ... do something
    return 'finished'

if __name__ == "__main__":
    # Create a ROS node for this state machine
    rospy.init_node("hbp_nrp_backend_sm_exp_control")

    sm = smach.StateMachine(
    outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
              'ACTION_PREEMPTED', 'ACTION_ERROR'])

    with sm:
        smach.StateMachine.add('CONDITION_30',
                               smach_ros.MonitorState('/ros_cle_simulation/status',
                                                      String,
                                                      condition_30_cb),
                               {'valid': 'CONDITION_30', 'invalid': 'ACTION_30',
                                'preempted': 'CONDITION_PREEMPTED'})
        smach.StateMachine.add('ACTION_30', CBState(action_30_cb),
                               {'finished': 'CONDITION_60'})
        smach.StateMachine.add('CONDITION_60',
                               smach_ros.MonitorState('/ros_cle_simulation/status',
                                                      String,
                                                      condition_60_cb),
                               {'valid': 'CONDITION_60', 'invalid': 'ACTION_60',
                                'preempted': 'CONDITION_PREEMPTED'})
        smach.StateMachine.add('ACTION_60', CBState(action_60_cb),
                               {'finished': 'FINISHED'})

    result = sm.execute()
    rospy.loginfo(result)
