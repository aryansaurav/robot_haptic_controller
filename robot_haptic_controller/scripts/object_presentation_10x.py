#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import LinkState
import PyKDL
import random
import time
from random import shuffle
from  math import pi
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.srv import SetLinkStateRequest



# topic = "/gazebo_11355/set_link_state"
topic = "/gazebo/set_link_state"

pub = rospy.Publisher(topic, LinkState, queue_size=1)
rospy.init_node('change_obj_pose', anonymous=True)

rospy.wait_for_service('/gazebo/set_link_state')
# rospy.wait_for_service('/gazebo_11355/set_link_state')
print("service is ready")

serv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
# serv = rospy.ServiceProxy('/gazebo_11355/set_link_state', SetLinkState)
state = LinkState()

numobj = range(3)
rotation_list = random.sample(numobj, len(numobj))
for j in range(3) :
	object_list = random.sample(numobj, len(numobj))
	for i in range(0,3):
		for k in range(50):
			print('\033[;40m  ')
		object_tograsp = object_list[i]
		state.pose.position.x = 0.5
		state.pose.position.y = 0.5
		state.pose.position.z = 0.5
		state.link_name = "cylinderHandle::left_wheel"
		sstate = SetLinkStateRequest()
		sstate.link_state = state
		serv(sstate)
		time.sleep(0.5)
		state.pose.position.x = 0.7
		state.pose.position.y = 0.7
		state.pose.position.z = 0.7
		state.link_name = "cross_joint_part::link"
		sstate = SetLinkStateRequest()
		sstate.link_state = state
		serv(sstate)
		time.sleep(0.5)
		state.pose.position.x = 0.9
		state.pose.position.y = 0.9
		state.pose.position.z = 0.9
		state.link_name = "thin_rod"
		sstate = SetLinkStateRequest()
		sstate.link_state = state
		serv(sstate)
		time.sleep(0.5)
		for reps in range(0,3) :
			if object_tograsp==0 :
				state.link_name = "cylinderHandle::left_wheel"
				state.pose.position.x = 0.05
				state.pose.position.y = 0
				state.pose.position.z = 0.15
				if rotation_list[j]==0 :
					x_rot = 0
					y_rot = (pi/2)-(pi/12)
					z_rot = pi/2
				elif rotation_list[j]==1 :
					x_rot = 0
					y_rot = (pi/2)+(pi/12)
					z_rot = pi/2
				else :
					x_rot = 0
					y_rot = pi/2
					z_rot = pi/2
				print(rotation_list[j])
			elif object_tograsp == 1 :
				state.link_name = "cross_joint_part::link"
				state.pose.position.x =0.17
				state.pose.position.y = -0.01
				state.pose.position.z = 0.1
				if rotation_list[j]==0 :
					x_rot = 0
					y_rot = 0
					z_rot = (pi/4) + (pi/12)
				elif rotation_list[j]==1 :
					x_rot = 0
					y_rot = 0
					z_rot = (pi/4) - (pi/12)
				else :
					x_rot = 0
					y_rot = 0
					z_rot = pi/4
				print(rotation_list[j])
			else :
				state.link_name = "thin_rod"
				state.pose.position.x = 0.1
				state.pose.position.y = 0.02
				state.pose.position.z = 0.15
				if rotation_list[j]==0 :
					x_rot = pi/2
					y_rot = (pi/2)+(pi/12)
					z_rot = 0
				elif rotation_list[j]==1 :
					x_rot = pi/2
					y_rot = (pi/2)-(pi/12)
					z_rot = 0
				else :
					x_rot = pi/2
					y_rot = pi/2
					z_rot = 0
				print(rotation_list[j])

			
			
			rot = PyKDL.Rotation.RPY(x_rot, y_rot, z_rot)
			quat = rot.GetQuaternion()

			state.pose.orientation.w = quat[3]
			state.pose.orientation.x= quat[0]
			state.pose.orientation.y= quat[1]
			state.pose.orientation.z= quat[2]

			sstate = SetLinkStateRequest()
			sstate.link_state = state

			serv(sstate)
			pub.publish(state)
			for k in range(50):
				print('\033[;42m  ')
			time.sleep(7)
			serv(sstate)
			pub.publish(state)
			for k in range(50):
				print('\033[;41m  ')
			time.sleep(7)

print('\033[0m')
#state.link_name = "cylinderHandle::left_wheel"
#state.link_name = "shapetoExplore3::sphere1"
# state.pose.position.x = -0.59
# state.pose.position.y = - 0.1
# state.pose.position.z = 0.07


# print(state.pose.position)
# #z_rot = random.uniform(-pi/4,pi/4)
# z_rot = pi/2
# print(z_rot)

# rot = PyKDL.Rotation.RPY(1.57, 0, z_rot)
# quat = rot.GetQuaternion()

# state.pose.orientation.w = quat[3]
# state.pose.orientation.x= quat[0]
# state.pose.orientation.y= quat[1]
# state.pose.orientation.z= quat[2]

# sstate = SetLinkStateRequest()
# sstate.link_state = state

# serv(sstate)


# x: -0.654324289579
# y: -0.0845832223655
# z: 0.07
# 0.764147923963
