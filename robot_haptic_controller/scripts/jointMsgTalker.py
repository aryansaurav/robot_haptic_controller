#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from my_msgs.msg import Mode
from math import pi
from numpy import deg2rad

def talker():
    pub2 = rospy.Publisher('trajectory', JointState, queue_size=10)
    pubMode = rospy.Publisher('mode', Mode, queue_size=10)
    rospy.init_node('talker', anonymous=True)


    # Primitives definition
    #OK
    pOfs123=dict()
    pOfs123['position']=[0,   0.872664626,   0.872664626,   0.785398163,   0,   0.872664626,  0.872664626,   0.785398163,   0,  0.872664626,   0.872664626,   0.785398163,  0.279252680,   0,   0,  0]
    pOfs123['patch1']=[0.,  0.,  0.,  0.,     0.,  0.,  0.,  0.,     0.,  0.,  0.,  0.,     1.1,  0.,  0.,  0.,     0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  1.]
    pOfs123['patch2']=[0. ,  0.7,  1. ,  1. ,  0. ,  0.7,  1. ,  1.1 ,  0. ,  0.7,  1. , 1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ]
    
    # OK (pb with this one)
    tsOfs123h=dict()
    tsOfs123h['position']=[0.        ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
    # tsOfs123h['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 1.,  1.1,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    # tsOfs123h['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    tsOfs123h['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  1.1,  0,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    tsOfs123h['patch2']=[0. ,  0.7,  1. ,  1. ,
                        0. ,  0.7,  1. ,  1.1 ,
                        0. ,  0.7,  1. , 1. ,
                        0. ,  0. , 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ]
    # OK
    tsOfs123l=dict()
    tsOfs123l['position']=[0.        ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
    tsOfs123l['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    # tsOfs123l['patch2']=[0.,  1.,  1.,  0.,  0.,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    tsOfs123l['patch2']=[0.,  1.,  1.,  0.,  0.,  1.1,  1.,  0.,  0.,  1.,  1.,  0.,  0., 0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]

    # OK
    tsOp=dict()
    tsOp['position']=[0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,  1.37881011, -0.1981317 , -0.17453293,  1.04719755]
    tsOp['patch1']=[0. ,  0. ,  0. ,  0. ,      0. ,  0. ,  0. ,  0. ,      0. ,  0. ,  0. , 0. ,      0. ,  0.7,  1. ,  1.1 ,      0. ,  0. ,  0. ,  0. ,  0. ,  0. , 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0.]
    tsOp['patch2']=[0.,  0.,  0.,  0.,          0.,  0.,  0.,  0.,          0.,  0.,  0.,  0.,         0., 0.,  0.,  0.,       0., 0.,  0.0,  0.0,     0.,  0.,  0.,  0.,     1.1,  0., 0.,  0.,     1.]
    #OK
    tsOs12=dict()
    tsOs12['position']=[0.30142573,  0.78539816,  1.04719755,  0.78539816,  0.30142573, 1.50098316,  0.87266463,  0.29670597,  0.        ,  0.2        ,  0.1        ,  0.1        ,  0.28, 0.22,-0.09,0.5]
    tsOs12['patch1']=[0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , 0. ,  0. ,  0.7,  1.1 ,  1. ,  0. ,  0.,  0. ,  0. ,  0. ,  0., 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0.]
    tsOs12['patch2']=[0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , 0. ,  0. ,  0.,  0. ,  0. ,  0. ,  0.7,  1.1 ,  1. ,  0. ,  0.7, 1. ,  1. ,  0. ,  0. ,  0. ,  0. ,  0.]
    #OK
    ttOft123=dict()
    ttOft123['position']=[0.        ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
    ttOft123['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    ttOft123['patch2']=[0.,  0.,  0.,  1.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  1.1,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
   

    #OK
    myGrasp=dict()
    myGrasp['position']=[0. ,  0.83775804,  0.34906585,  0.12217305,  0. ,  0.83775804,  0.34906585,  0.12217305,  0.        ,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
    myGrasp['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    myGrasp['patch2']=[0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.0,  0.,  0.,  0.,  1.1,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]

    mydeg2rad = lambda y: map(lambda x:x*pi/180, y)

    # primitives = []
    # primitives.append(ttOft123) #0 TIP
    # primitives.append(tsOfs123h)#1 SURF H
    # primitives.append(tsOfs123l)#2 SURF L
    # primitives.append(pOfs123)  #3 PALM 123
    # primitives.append(tsOp)     #4 PALM T
    # primitives.append(tsOs12)   #5 SIDE
    # primitives.append(myGrasp)  #6 SIDE

    # ### Choose the desired primitive
    # # chosen_primitive = primitives[5]
    # chosen_primitive = primitives[6]
    # # Trying tests with ravin
    chosen_primitive = tsOfs123h


    #prim 2 position
    deg2rad = pi/180
    some_robot_position_prim2 = [88.7*deg2rad,-39.2*deg2rad,-112.67*deg2rad,-82.9*deg2rad,35*deg2rad,2.05*deg2rad,21.49*deg2rad] # some vertical position


    # some_robot_position = [2.75,0.6,-2.9,2.0,2.95,-1.0,-2.9]
    # some_robot_position = [2.4271998405456543, 0.6363900303840637, 0.6379094123840332, -1.992479920387268, 0.04142158105969429, -0.6957219243049622, 2.9567058086395264]
    # try without touching, 1
    # some_robot_position = [2.559328079223633, 0.582476794719696, 0.4580463469028473, -1.7711386680603027, 0.5149389505386353, -0.03630857914686203, 2.3363845348358154]
    # 

    some_robot_position = [2.0513858795166016, 0.9230818152427673, 1.0374879837036133, -1.9326180219650269, -0.11087016761302948, -0.5212043523788452, 2.390393018722534]
    some_robot_position = [0.47, 0.9230818152427673, 1.0374879837036133, -1.9326180219650269, -0.11087016761302948, -0.5212043523788452, 2.390393018722534]
    # Better position for joint limits (more or less ...)
    some_robot_position =    [2.1136364936828613, 1.7524466514587402, -2.194305181503296, -1.759585976600647, -0.5912100672721863, -1.5606279373168945, -0.8991972208023071]

# some_robot_position = [2.7054507732391357, 0.9147441983222961, 0.5324338674545288, -1.8639016151428223, 0.3882056474685669, -0.8152718544006348, 2.4347145557403564 ]
    # some_robot_position = [0,1.5,0,0,0,0,0]
    # # Some vertical position for the hand
    # some_robot_position = [85*deg2rad,-78*deg2rad,-9*deg2rad,-73*deg2rad,36*deg2rad,8*deg2rad,3*deg2rad] # some vertical position
    # # More horizontal position
    # some_robot_position = [77*deg2rad,-54*deg2rad,-95*deg2rad,-84*deg2rad,87*deg2rad,-60*deg2rad,2*deg2rad] # some vertical position
    # some_robot_position = [97*deg2rad,-30*deg2rad,-95*deg2rad,-84*deg2rad,87*deg2rad,-60*deg2rad,2*deg2rad] # some vertical position
    # #ravin chose that position
    # some_robot_position = [103*deg2rad,-30*deg2rad,-120*deg2rad,-73*deg2rad,125*deg2rad,-50*deg2rad,-10*deg2rad] # some vertical position
    # some_robot_position = [91*deg2rad,-33*deg2rad,-113*deg2rad,-65*deg2rad,131*deg2rad,-38*deg2rad,-10*deg2rad] # some vertical position

    #Choose the arm's configuration
    # some_robot_position=some_robot_position_prim2

    # TODO: Find a new arm configuration



    zero_fingers = [0,0,0,0,  0,0,0,0,  0,0,0,0,  0.4,0,0,0]

    joint_state_zero = JointState(position =some_robot_position + zero_fingers)

    for count in range(0,5):
        pub2.publish(joint_state_zero)
        time.sleep(0.1)

    time.sleep(2.0)



    for count in reversed(range(0,5)):

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)



        # USE THIS FOR SIM (very close to joint limits ...)
        # joint_state = JointState(position =[2.75,0.6,-2.9,2.0,2.95,-1.0,-2.9,     0,0.7,0.2,0.2, 0,0.7,0.2,0.2, 0,0.7,0.2,0.2, 1.57,0,0.17,0.09 ])

        ## USE THIS FOR REAL ROBOT
        # joint_state = JointState(position =[-0.9, 1.43,-1.74, 0.9,0.36, -0.28, 2.70,        0,0.7,0.2,0.2, 0,0.7,0.2,0.2, 0,0.7,0.2,0.2, 1.57,0,0.17,0.09 ])

        # New ravin's data
        # joint_state = JointState(position =[2.75,0.6,-2.9,2.0,2.95,-1.0,-2.9,       0.40142573,  0.6981317 ,  1.04719755,  0.78539816,  0.40142573, 1.41371669,  0.87266463,  0.29670597,  0.        ,  0.        , 0.        ,  0.        ,  0.61086524, -0.59341195,  0.17453293, 0.34906585])
        joint_state = JointState(position = some_robot_position + chosen_primitive['position'])


        b_send_trajectory = True
        # b_send_trajectory = False
        if b_send_trajectory:
            pub2.publish(joint_state)
            rospy.loginfo("Sending traj")




        b_send_patches = True
        # b_send_patches = False
        if b_send_patches:
            # Send desired patches
            mode = Mode()
            mode.mode = True
            # mode.patch1 = [0,0.3,0.3,0.3, 0,0.5,0.3,1.0, 0,0,0.3,0.4, 0,0,0,0]
            # mode.patch2 = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0.3,1]

            # length 29
            mode.patch1 = chosen_primitive['patch1']
            mode.patch2 = chosen_primitive['patch2']

            pubMode.publish(mode)

            rospy.loginfo("Sending patches: " + str(count))

            time.sleep(0.1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
