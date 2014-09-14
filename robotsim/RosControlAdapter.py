from robotsim.RobotInterface import IRobotControlAdapter
import rospy, sys, os, time
import string
import warnings

from gazebo_ros import gazebo_interface

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench

__author__ = 'NinoCauli'


class RosControlAdapter(IRobotControlAdapter):
    def initialize(self):
        get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physics = get_physics_properties()
        paused = physics.pause
        if (not paused):
            rospy.wait_for_service('/gazebo/pause_physics')
            pause_client = rospy.ServiceProxy('gazebo/pause_physics', Empty)
            pause_client()
        reset = rospy.ServiceProxy('gazebo/reset_sim', Empty)
        reset()
        pass

    def is_paused(self):
        get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physics = get_physics_properties()
        paused = physics.pause
        return paused

    def is_alive(self):
        get_world_properties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
        world = get_world_properties()
        success = world.success
        return success

    def run_step(self, dt):
        advance_simulation = rospy.ServiceProxy('gazebo/advance_simulation', AdvanceSimulation)
        advance_simulation(dt)  
        get_world_properties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
        world = get_world_properties()
        simTime = world.sim_time     
        return simTime

    def shutdown(self):
        # TODO test better the endWorld function recently added in the gazebo_ros_HBP plugin
        
        # endWorld = rospy.ServiceProxy('gazebo/end_world', Empty)
        # endWorld()
        pass
    