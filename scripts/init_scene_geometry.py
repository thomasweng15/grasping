#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 29 14:05:28 2018

@author: bokorn
"""
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import intera_interface
import numpy as np
from sawyer_utils.sawyer_joint_control import SawyerJointControl
import sawyer_utils.moveit_scene_utils as scene_utils
import tf.transformations as trans

base_standoff = 0.2
sensor_standoff_z = 0.1
sensor_standoff_y = 0.2

def initScene(scene, table_shape = [0.7, 1.6], table_height = -0.17):
    scene.remove_world_object()

    time_stamp = rospy.get_rostime()
    
    table_center = [base_standoff + table_shape[0]/2.0, 0.0, table_height]    
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base'
    table_pose.header.stamp = time_stamp
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = table_center[0]
    table_pose.pose.position.y = table_center[1]
    table_pose.pose.position.z = table_center[2]
    table_size =  tuple(table_shape) + (0.0, )

    ceiling_size = [table_size[1], table_size[0], table_size[2]]
    ceiling_pose = PoseStamped()
    ceiling_pose.header.frame_id = 'primesense'
    ceiling_pose.header.stamp = time_stamp
    ceiling_pose.pose.orientation.w = 1.0
    ceiling_pose.pose.position.x = 0.0
    ceiling_pose.pose.position.y = sensor_standoff_y
    ceiling_pose.pose.position.z = sensor_standoff_z

    wall_height = 2.0
    wall_b_angle = -0.40970294454245623
    
    wall_b_size = (0.0, 1.05, wall_height)
    wall_bl_pose = PoseStamped()
    wall_bl_pose.header.frame_id = 'base'
    wall_bl_pose.header.stamp = time_stamp
    wall_bl_quat = trans.quaternion_about_axis(wall_b_angle, (0,0,1))
    wall_bl_pose.pose.orientation.x = wall_bl_quat[0]
    wall_bl_pose.pose.orientation.y = wall_bl_quat[1]
    wall_bl_pose.pose.orientation.z = wall_bl_quat[2]
    wall_bl_pose.pose.orientation.w = wall_bl_quat[3]
    wall_bl_pose.pose.position.x = 0.0
    wall_bl_pose.pose.position.y = 0.5
    wall_bl_pose.pose.position.z = wall_height/2.0 + table_center[2]
    wall_br_pose = PoseStamped()
    wall_br_pose.header.frame_id = 'base'
    wall_br_pose.header.stamp = time_stamp
    wall_br_quat = trans.quaternion_about_axis(-wall_b_angle, (0,0,1))
    wall_br_pose.pose.orientation.x = wall_br_quat[0]
    wall_br_pose.pose.orientation.y = wall_br_quat[1]
    wall_br_pose.pose.orientation.z = wall_br_quat[2]
    wall_br_pose.pose.orientation.w = wall_br_quat[3]
    wall_br_pose.pose.position.x = 0.0
    wall_br_pose.pose.position.y = -0.5
    wall_br_pose.pose.position.z = wall_height/2.0 + table_center[2]
    
    wall_s_size = (table_size[0], 0.0, wall_height)
    wall_sl_pose = PoseStamped()
    wall_sl_pose.header.frame_id = 'base'
    wall_sl_pose.header.stamp = time_stamp
    wall_sl_pose.pose.orientation.w = 1
    wall_sl_pose.pose.position.x = table_center[0]
    wall_sl_pose.pose.position.y = table_size[1]/2.0
    wall_sl_pose.pose.position.z = wall_height/2.0 + table_center[2]
    wall_sr_pose = PoseStamped()
    wall_sr_pose.header.frame_id = 'base'
    wall_sr_pose.header.stamp = time_stamp
    wall_sr_pose.pose.orientation.w = 1
    wall_sr_pose.pose.position.x = table_center[0]
    wall_sr_pose.pose.position.y = -table_size[1]/2.0
    wall_sr_pose.pose.position.z = wall_height/2.0 + table_center[2]

    wall_f_size = (0.0, table_size[1], wall_height)
    wall_f_pose = PoseStamped()
    wall_f_pose.header.frame_id = 'base'
    wall_f_pose.header.stamp = time_stamp
    wall_f_pose.pose.orientation.w = 1
    wall_f_pose.pose.position.x = table_center[0] + table_size[0]/2.0
    wall_f_pose.pose.position.y = 0.0
    wall_f_pose.pose.position.z = wall_height/2.0 + table_center[2]

    rospy.sleep(0.2)
    # scene.add_box('table', table_pose, table_size)
    scene.add_box('ceiling', ceiling_pose, ceiling_size)
    # scene.add_box('wall_f', wall_f_pose, wall_f_size)
    # scene.add_box('wall_bl', wall_bl_pose, wall_b_size)
    # scene.add_box('wall_br', wall_br_pose, wall_b_size)    
    # scene.add_box('wall_sl', wall_sl_pose, wall_s_size)
    # scene.add_box('wall_sr', wall_sr_pose, wall_s_size)
    rospy.sleep(0.1)

try:
    rospy.init_node("scene_geometry")
    scene = moveit_commander.PlanningSceneInterface()
    
    sawyer = SawyerJointControl(scene)
    initScene(scene, 
                table_shape=[1.0, 1.6],
                table_height=-0.21)

    # scene_utils.addGripperObject(scene, 
    #                              object_size=[.05,.05,.12], 
    #                              gripper_frame = 'right_hand')
    
    scene_utils.addGripperObject(scene, 
                                 object_size=[.19,.19,.1], 
                                 object_center= [0.0,0.0,.31], 
                                 object_name='realsense', 
                                 gripper_frame='head')

    
    import IPython; IPython.embed()
except rospy.ROSInterruptException:
    pass
