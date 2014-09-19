from python_cle.robotsim.RobotInterface import IRobotControlAdapter
import rospy

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty

__author__ = 'NinoCauli'


class RosControlAdapter(IRobotControlAdapter):
    def initialize(self):
        rospy.wait_for_service('/gazebo/get_physics_properties')
        self.__get_physics_properties = rospy.ServiceProxy(
                      'gazebo/get_physics_properties', GetPhysicsProperties)
        rospy.wait_for_service('/gazebo/get_world_properties')
        self.__get_world_properties = rospy.ServiceProxy(
                      'gazebo/get_world_properties', GetWorldProperties)
        rospy.wait_for_service('/gazebo/set_physics_properties')
        self.__set_physics_properties = rospy.ServiceProxy(
                      'gazebo/set_physics_properties', SetPhysicsProperties)
        rospy.wait_for_service('/gazebo/pause_physics')
        self.__pause_client = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        rospy.wait_for_service('/gazebo/reset_sim')
        self.__reset = rospy.ServiceProxy('gazebo/reset_sim', Empty)
        rospy.wait_for_service('gazebo/end_world')
        self.__endWorld = rospy.ServiceProxy('gazebo/end_world', Empty)
        rospy.wait_for_service('gazebo/advance_simulation')
        self.__advance_simulation = rospy.ServiceProxy(
                       'gazebo/advance_simulation', AdvanceSimulation)
        physics = self.__get_physics_properties()
        paused = physics.pause
        if (not paused):
            self.__pause_client()
        self.__reset()
        self.get_time_step()
        pass

    def get_time_step(self):
        physics = self.__get_physics_properties()
        self.__time_step = physics.time_step
        return self.__time_step

    def set_time_step(self, time_step):
        physics = self.__get_physics_properties()
        success = self.__set_physics_properties(
            time_step,
            physics.max_update_rate,
            physics.gravity,
            physics.ode_config)
        if (success):
            self.__time_step = time_step
        return success

    def is_paused(self):
        physics = self.__get_physics_properties()
        paused = physics.pause
        return paused

    def is_alive(self):
        world = self.__get_world_properties()
        success = world.success
        return success

    def run_step(self, dt):
        if dt % self.__time_step == 0:
            steps = dt / self.__time_step
            self.__advance_simulation(steps)
            world = self.__get_world_properties()
            simTime = world.sim_time
        else:
            simTime = -1
            raise ValueError("dt is not multiple of the physics time step")
        return simTime

    def shutdown(self):
        self.__endWorld()
        pass
