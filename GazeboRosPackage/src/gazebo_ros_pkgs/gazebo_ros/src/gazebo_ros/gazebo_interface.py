#! /usr/bin/env python2
"""Gazebo Interface"""
# Wrappers around the services provided by rosified gazebo

# import sys
import rospy
# import os

# pylint: disable=W0401
# pylint: disable=W0614
# pylint: disable=W0611
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


def spawn_sdf_model_client(
    model_name, model_xml, robot_namespace, initial_pose, reference_frame,
        gazebo_namespace):
    """function that spawn sdf models"""
    rospy.loginfo("Waiting for service %s/spawn_sdf_model" % gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace + '/spawn_sdf_model')
    try:
        spawn_sdf_model = rospy.ServiceProxy(
            gazebo_namespace + '/spawn_sdf_model', SpawnModel)
        rospy.loginfo("Calling service %s/spawn_sdf_model" % gazebo_namespace)
        resp = spawn_sdf_model(
            model_name, model_xml, robot_namespace, initial_pose,
            reference_frame)
        rospy.loginfo("Spawn status: %s" % resp.status_message)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def spawn_urdf_model_client(
    model_name, model_xml, robot_namespace, initial_pose, reference_frame,
        gazebo_namespace):
    """function that spawn urdf models"""
    rospy.loginfo("Waiting for service %s/spawn_urdf_model" % gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace + '/spawn_urdf_model')
    try:
        spawn_urdf_model = rospy.ServiceProxy(
            gazebo_namespace + '/spawn_urdf_model', SpawnModel)
        rospy.loginfo("Calling service %s/spawn_urdf_model" % gazebo_namespace)
        resp = spawn_urdf_model(
            model_name, model_xml, robot_namespace, initial_pose,
            reference_frame)
        rospy.loginfo("Spawn status: %s" % resp.status_message)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def set_model_configuration_client(
    model_name, model_param_name, joint_names, joint_positions,
        gazebo_namespace):
    """function that sets the model configuration"""
    rospy.loginfo("Waiting for service %s/set_model_configuration" %
                  gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace + '/set_model_configuration')
    try:
        set_model_configuration = rospy.ServiceProxy(
            gazebo_namespace + '/set_model_configuration',
            SetModelConfiguration)
        rospy.loginfo("Calling service %s/set_model_configuration" %
                      gazebo_namespace)
        resp = set_model_configuration(
            model_name, model_param_name, joint_names, joint_positions)
        rospy.loginfo("Set model configuration status: %s" %
                      resp.status_message)

        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
