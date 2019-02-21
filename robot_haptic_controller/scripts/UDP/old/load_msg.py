#!/usr/bin/env python

import pickle
import sys
import struct
import numpy as np

import rospy
from sensor_msgs.msg import JointState
import time
import math
# print(math.pi)

# import receive_udp_test_EMG.py
data = pickle.load(open("save_msg_good_values.p", "rb"))

# Store finger joint names (map ?)
rospy.init_node('emg_bridge', anonymous=True)
# pub = rospy.Publisher('/desired_hand_state', JointState, queue_size=0)
pub = rospy.Publisher('/allegroHand_1/joint_states', JointState, queue_size=0)
# print("joint_{}.0".format(1))

# Give some time for everything to be initialized
time.sleep(0.1)
names = ["joint_{}.0".format(x) for x in range(0, 16)]
print(names)
# names.append('ahand/thumb/dof0_joint')


def get_positions_from_raw_data(data):
    joint_vector = []
    for joint in range(0, 27):
        joint_vector.append(struct.unpack("f", data[4 + 4 * joint:4 + 4 * joint + 4])[0])
    return joint_vector


def publish_EMG_positions(emg_full_joints):
    EMG_finger_joints = emg_full_joints[6:-1]
    hand_state = np.zeros(16)
    # hand
    # index
    hand_state[0] = EMG_finger_joints[0]
    hand_state[1] = EMG_finger_joints[1]
    hand_state[2] = EMG_finger_joints[2]
    hand_state[3] = EMG_finger_joints[3]
    # middle
    hand_state[4] = EMG_finger_joints[4]
    hand_state[5] = EMG_finger_joints[5]
    hand_state[6] = EMG_finger_joints[6]
    hand_state[7] = EMG_finger_joints[7]
    # pinky --> skipping 1 joint ...
    hand_state[8] = EMG_finger_joints[12]
    hand_state[9] = EMG_finger_joints[13]
    hand_state[10] = EMG_finger_joints[14]
    hand_state[11] = EMG_finger_joints[15]
    # thumb
    hand_state[12] = EMG_finger_joints[16]
    hand_state[13] = EMG_finger_joints[17]
    hand_state[14] = EMG_finger_joints[18]
    hand_state[15] = EMG_finger_joints[19]

    hand_state_msg = JointState(position=hand_state, name=names)
    hand_state_msg.header.stamp = rospy.Time.now()
    pub.publish(hand_state_msg)
    print("published something")


for i in range(1, 5):
    full_joints = get_positions_from_raw_data(data)
    publish_EMG_positions(full_joints)
    time.sleep(0.5)
