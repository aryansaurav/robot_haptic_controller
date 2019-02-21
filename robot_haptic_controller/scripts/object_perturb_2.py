#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray

def init_perturb():
	b_isRobot = False

	if b_isRobot:
	    topic = '/grasping_perturbation'
	else:
	    topic = '/object_plugin/reference_update'
	   
	global pub    
	pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
	rospy.init_node('talker', anonymous=True)

	global array
	array = Float32MultiArray()

	global perturbations

	perturbations= []
	zero_perturbation = [0,0,0 , 0,0,0]
	perturbations.append(zero_perturbation)
	perturbations.append([0,0,-0.02, 0,0,0])
	perturbations.append(zero_perturbation)
	perturbations.append([0.0,0.02,0.0, 0,0,0])
	perturbations.append(zero_perturbation)

	perturbations.append([0,0,0, 0.7,0,0])
	perturbations.append(zero_perturbation)
	perturbations.append([0,0,0, -0.7,0,0])
	perturbations.append(zero_perturbation)
	# perturbations.append([0,0,0, 0,0.,0.3])
	perturbations.append([0,0,0, 0,0.7,0])
	perturbations.append(zero_perturbation)

	# Take the negative of each value:
	perturbations = [[-value for value in perturbation] for perturbation in perturbations]

	print(perturbations)
	array.data = zero_perturbation
	# array.data = [0.08,0,0.05, 0,0,0]
	pub.publish(array)


def perturb(index):
	#Reset index
	#index = 0


	# Send each perturbation
	array.data = perturbations[index]
	pub.publish(array)
	print("published array: " + str(array.data))
	time.sleep(2)
	#index += 1



if __name__ == '__main__':
	try:
		init_perturb()
		for index in range(0,11):
		 	perturb(index)

	except rospy.ROSInterruptException:
	    pass