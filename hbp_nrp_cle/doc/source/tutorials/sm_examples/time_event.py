#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String
from gazebo_msgs.srv import SetVisualProperties
from gazebo_msgs.srv import SetVisualPropertiesRequest


def time_30secs_cb(user_data, status):

    # ... parse JSON in status message ...

    if status.simulationTime < 30:
        return True  # condition not met so far, evaluate state outcome to still 'valid'
    else:
        return False  # condition met, evaluates state outcome to 'invalid'


if __name__ == "__main__":
    # Create a ROS node for this state machine
    rospy.init_node("hbp_nrp_backend_sm_exp_control")

    sm = smach.StateMachine(
    outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
              'ACTION_PREEMPTED', 'ACTION_ERROR'])

    with sm:
        smach.StateMachine.add('CONDITION',
                               smach_ros.MonitorState('/ros_cle_simulation/status',
                                                      String,
                                                      time_30secs_cb),
                               {'valid': 'CONDITION', 'invalid': 'ACTION',
                                'preempted': 'CONDITION_PREEMPTED'})
        smach.StateMachine.add('ACTION',
                               smach_ros.ServiceState('/gazebo/set_visual_properties',
                                                      SetVisualProperties,
                                                      request=SetVisualPropertiesRequest(
                                                          'right_vr_screen', 'body', 'screen_glass',
                                                          'material:script:name', 'Gazebo/Red')),
                               {'succeeded': 'FINISHED', 'aborted': 'ACTION_ERROR',
                                'preempted': 'ACTION_PREEMPTED'})

    result = sm.execute()
    rospy.loginfo(result)
