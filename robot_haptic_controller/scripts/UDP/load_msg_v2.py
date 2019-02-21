#!/usr/bin/env python
""" This file receives joints from the Geneva teams through UDP.
The joints 

"""
import pickle
import sys
import struct
import numpy as np

import rospy
from sensor_msgs.msg import JointState
import time
import socket
import math


def map_emg_to_allegro_0(emg_joints):
    allegro_joints = np.zeros(16)
    # index
    allegro_joints[0] = emg_joints[0]
    allegro_joints[1] = emg_joints[1]
    allegro_joints[2] = emg_joints[2]
    allegro_joints[3] = emg_joints[3]
    # middle
    allegro_joints[4] = emg_joints[4]
    allegro_joints[5] = emg_joints[5]
    allegro_joints[6] = emg_joints[6]
    allegro_joints[7] = emg_joints[7]
    # pinky --> skipping 1 joint ...
    allegro_joints[8] = emg_joints[12]
    allegro_joints[9] = emg_joints[13]
    allegro_joints[10] = emg_joints[14]
    allegro_joints[11] = emg_joints[15]
    # thumb
    allegro_joints[12] = emg_joints[16]
    allegro_joints[13] = emg_joints[17]
    allegro_joints[14] = emg_joints[18]
    allegro_joints[15] = emg_joints[19]
    return allegro_joints


def map_emg_to_allegro_1(emg_joints):
    allegro_joints = np.zeros(16)
    # index
    allegro_joints[0] = emg_joints[0]
    allegro_joints[1] = emg_joints[1]
    allegro_joints[2] = emg_joints[2]
    allegro_joints[3] = emg_joints[3]
    # middle: get it from the index
    allegro_joints[4] = emg_joints[0]
    allegro_joints[6] = emg_joints[1]
    allegro_joints[5] = emg_joints[2]
    allegro_joints[7] = emg_joints[3]
    # pinky --> skipping 1 joint ...
    allegro_joints[8] = emg_joints[12]
    allegro_joints[9] = emg_joints[13]
    allegro_joints[10] = emg_joints[14]
    allegro_joints[11] = emg_joints[15]
    # thumb
    allegro_joints[12] = emg_joints[16]
    allegro_joints[13] = emg_joints[17]
    allegro_joints[14] = emg_joints[18]
    allegro_joints[15] = emg_joints[19]
    return allegro_joints


def map_emg_to_allegro_2(emg_joints):
    allegro_joints = np.zeros(16)
    # index
    allegro_joints[0] = emg_joints[0]
    allegro_joints[1] = emg_joints[1]
    allegro_joints[2] = emg_joints[2]
    allegro_joints[3] = emg_joints[3]
    # middle: get it from the index
    allegro_joints[4] = emg_joints[4]
    allegro_joints[5] = emg_joints[5]
    # allegro_joints[5] = emg_joints[5] - 1.0# it used to switch 5 and 6, not sure if desired
    allegro_joints[6] = emg_joints[6]
    allegro_joints[7] = emg_joints[7]
    # pinky --> skipping 1 joint ...
    allegro_joints[8] = emg_joints[12]
    allegro_joints[9] = emg_joints[13]
    allegro_joints[10] = emg_joints[14]
    allegro_joints[11] = emg_joints[15]
    # # thumb
    # allegro_joints[12] = emg_joints[16]
    # allegro_joints[13] = emg_joints[17]
    # allegro_joints[14] = emg_joints[18]
    # allegro_joints[15] = emg_joints[19]
    # thumb
    allegro_joints[12] = emg_joints[16] 
    allegro_joints[13] = -0.1
    allegro_joints[14] = emg_joints[18]  # emg_joints[18]
    allegro_joints[15] = emg_joints[19]
    return allegro_joints

def map_emg_to_allegro_3(emg_joints):
    allegro_joints = np.zeros(16)
    # index
    allegro_joints[0] = 0.0
    allegro_joints[1] = (emg_joints[1]) 
    allegro_joints[2] = (emg_joints[2]) 
    allegro_joints[3] = emg_joints[3]
    # middle
    allegro_joints[4] = 0.0
    allegro_joints[5] = emg_joints[5]
    allegro_joints[6] = emg_joints[6]
    allegro_joints[7] = emg_joints[7]

    # ring 
    allegro_joints[8] = 0.0
    allegro_joints[9] =  emg_joints[9]
    allegro_joints[10] = emg_joints[10]
    allegro_joints[11] = emg_joints[11]
    
    # pinky --> skipping 1 joint ...
    # allegro_joints[8] = 0.0
    # allegro_joints[9] = emg_joints[13]
    # allegro_joints[10] = emg_joints[14]
    # allegro_joints[11] = emg_joints[15]

    # thumb
    allegro_joints[12] = emg_joints[16]  # go more into "opposition" configuration
    allegro_joints[13] = emg_joints[17]
    allegro_joints[14] = emg_joints[18] 
    allegro_joints[15] = emg_joints[19]
    return allegro_joints

class emg_bridge(object):
    """connect to UDP to get decoded joint values from
    EMG. Do some light processing and send in a ROS topic
    """

    def __init__(self):
        rospy.init_node('emg_bridge', anonymous=True)
        # Either send the commands to display in Rviz, or directly as desired state ...
        # rviz_display = True  # Just display the desired pose in Rviz, or send it as desired pose
        rviz_display = False
        if not rviz_display:
            self.pub = rospy.Publisher('/desired_hand_state', JointState, queue_size=0)
        else:
            self.pub = rospy.Publisher('/allegroHand_1/joint_states', JointState, queue_size=0)

        time.sleep(0.1)
        # Store finger joint names
        self.names = ["joint_{}.0".format(x) for x in range(0, 16)]

    def start_connection(self):

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the port
        # self.server_address = ('128.178.145.174', 9027)
        # self.server_address = ('128.178.145.250', 9027)
        #self.server_address = ('10.16.13.38', 9027)    # CHUV
        #self.server_address = ('128.178.51.82', 9027)  # campus biotech
        #self.server_address = ('128.178.145.170', 9024)  # campus biotech
        self.server_address = ('128.178.145.170', 9024)  # LASA

        # self.server_address = ('192.168.0.122', 9024) # Rome
        #self.server_address = ('192.168.0.29', 9027)   # Rome
        #self.server_address = ('10.1.7.83', 9027)      # Villa Beretta
        #self.server_address = ('128.179.191.244', 9024)   # EPFL (lefty)
        #self.server_address = ('128.179.191.244', 9027)  # EPFL (righty)

        print >>sys.stderr, 'starting up on %s port %s' % self.server_address
        self.sock.bind(self.server_address)

        while True:
            print >>sys.stderr, '-------\nwaiting to receive message'
            data, address = self.sock.recvfrom(4096)
            full_joints = self.get_positions_from_raw_data(data)
            self.publish_EMG_positions(full_joints)

            np.set_printoptions(precision=3)
            print(full_joints)
            print ((np.array(full_joints[7:27]).reshape(5, 4)))

    def get_positions_from_raw_data(self, data):
        joint_vector = []
        for joint in range(0, 27):
            joint_vector.append(struct.unpack("f", data[4 + 4 * joint:4 + 4 * joint + 4])[0])
        return joint_vector

    def publish_EMG_positions(self, emg_full_joints):
        EMG_finger_joints = emg_full_joints[7:27]
        hand_state = np.zeros(16)

        # hand_state = map_emg_to_allegro_1(EMG_finger_joints)
        # hand_state = map_emg_to_allegro_2(EMG_finger_joints)
        hand_state = map_emg_to_allegro_3(EMG_finger_joints)

        receive_degrees = False
        if receive_degrees:
            hand_state = [angle * math.pi / 180 for angle in hand_state]

        hand_state_msg = JointState(position=hand_state, name=self.names)
        hand_state_msg.header.stamp = rospy.Time.now()
        self.pub.publish(hand_state_msg)
        print("published something")

    def start_demo_pickle(self):
        data = pickle.load(open("save_msg_good_values.p", "rb"))

        # send the data 5 times
        for i in range(1, 5):
            full_joints = self.get_positions_from_raw_data(data)
            self.publish_EMG_positions(full_joints)
            time.sleep(0.5)


if __name__ == '__main__':
    try:
        new = emg_bridge()

        # Demo means we replay some recorded data.
        demo = False
        # demo = True
        if demo:
            new.start_demo_pickle()
        else:
            new.start_connection()
        # while True:
        # while not rospy.is_shutdown():
        #     print("bla")
        #     time.sleep(0.5)



    except rospy.ROSInterruptException:
        print("ros interrupt exception")
        pass
