def condition_spike_cb(user_data, spike):
    # ... parse JSON in spike message ...
    if spike < 10:
        return True  # condition not met so far, evaluate state outcome to still 'valid'
    else:
        return False  # condition met, evaluates state outcome to 'invalid'


if __name__ == "__main__":
    # ...

        smach.StateMachine.add('CONDITION',
                               smach_ros.MonitorState('/monitoring/left_wheel_neuron_rate_monitor',
                                                      String,
                                                      condition_spike_cb),
                               {'valid': 'CONDITION', 'invalid': 'ACTION',
                                'preempted': 'CONDITION_PREEMPTED'})
