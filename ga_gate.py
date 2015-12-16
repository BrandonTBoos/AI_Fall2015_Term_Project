#!/usr/bin/env python

import sys
import rospy
import gazebo_ros
from std_srvs.srv import Empty
from gazebo_msgs.srv import *


def reset_model():
	rospy.wait_for_service('/gazebo/reset_simulation')
	try:
		reset_model = rospy.ServiceProxy('/gazebo/reset_simulation', Empty )
		reset_model()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def apply_torque(torque,joint):
	print 'reset'
	clear_torque(joint)
	print 'cleared'
	add_torque(torque,joint)
	print 'done'

def clear_torque(joint):
	rospy.wait_for_service('/gazebo/clear_joint_forces')
	try:
		print 1
		reset_torque = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
		print 2
		reset_torque(joint)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	

def add_torque(torque,joint):
	rospy.wait_for_service('/gazebo/apply_joint_effort')
	try:
		reset_torque = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort )
		reset_torque(joint,torque,rospy.Time(),rospy.Duration(10))
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	

if __name__ == "__main__":
	reset_model()
	apply_torque(10,'leg_1_joint')
