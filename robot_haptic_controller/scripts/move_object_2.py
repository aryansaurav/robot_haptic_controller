#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import LinkState
import PyKDL
import random
from random import randint
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
#state.link_name = "cylinderHandle::left_wheel"
state.link_name = "shapetoExplore3::sphere1"
# state.pose.position.x = -0.59
# state.pose.position.y = - 0.1
# state.pose.position.z = 0.07
x = -0.59
y = -0.1
z = 0.07
x_range = 0.1
y_range = 0.1
state.pose.position.x = random.uniform(x-x_range,x+x_range)
state.pose.position.y = random.uniform(y-y_range,y+y_range)
state.pose.position.z = z
print(state.pose.position)
z_rot = random.uniform(-pi/4,pi/4)
print(z_rot)

rot = PyKDL.Rotation.RPY(1.57, 0, z_rot)
quat = rot.GetQuaternion()

state.pose.orientation.w = quat[3]
state.pose.orientation.x= quat[0]
state.pose.orientation.y= quat[1]
state.pose.orientation.z= quat[2]

sstate = SetLinkStateRequest()
sstate.link_state = state

serv(sstate)


# for _ in range(0,10):
#     pub.publish(state)
#     rospy.sleep(0.15)


# x: -0.654324289579
# y: -0.0845832223655
# z: 0.07
# 0.764147923963
