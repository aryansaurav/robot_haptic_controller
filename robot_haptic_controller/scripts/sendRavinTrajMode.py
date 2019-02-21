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
    tsOfs123h['patch1']=[0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    tsOfs123h['patch2']=[0. ,  0.7,  1. ,  1. ,  0. ,  0.7,  1. ,  1.1 ,  0. ,  0.7,  1. , 1. ,  0. ,  0. , 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , 0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ]
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

    # TSoS
    TSoS_1 =  [-99.2948431,    61.29151285,  -93.22621993,  74.04275265 ,   0. ,-43.35548385 , 124.72066127]
    TSoS_1_rad = [-1.754062533378601, 1.0913132429122925, -1.6341289281845093, 1.3748565912246704, -0.024337133392691612, -0.691257894039154, 2.1780526638031006]

    TSoS_2 =  [-24.7583023 ,   59.8517294  , 105.47338859,   26.92354415,    0. , -96.55955546,   37.86257631]
    TSoS_2_rad = [-0.38407838344573975, 1.075904369354248, 1.8563232421875, 0.45743533968925476, -0.14451324939727783, -1.6516625881195068, 0.6274010539054871]

    TSoS_3 =  [-5.03650836 , 86.92543545 , 68.42338441,  63.17391673 ,  0.,         -63.15096294 ,84.0646978]
    TSoS_3_rad =[0.11177311837673187, 1.2919540405273438, 1.451442003250122, 1.4155633449554443, -0.05327209085226059, -0.6658159494400024, 1.6641792058944702]

    TSoP = [-72.04233354 ,  66.07972463 ,-108.79614176 ,  67.29212777 ,   0., 104.51662599 ,   0.67156015] 
    TSoP_rad = [-1.2707359790802002, 1.1641528606414795, -1.7772085666656494, 1.281318187713623, 0.37211906909942627, 1.643073320388794, 0.007252087350934744]

    PoFS = [-85.7595384 ,  93.98048153, -58.2700355 ,  61.86579764  , 0.      ,   -16.68919887 , -14.18654632] 
    PoFS_rad = [-1.382082462310791, 1.4727118015289307, -1.3505675792694092, 1.4374736547470093, -0.031195037066936493, 1.6256251335144043, -0.28351056575775146]

    TSoFS = [-81.46269043, 93.59863862, -60.53253486 , 49.4618776  ,  0.  ,       -33.2704934 , -5.50911146] 
    TSoFS_rad = [-0.9988199472427368, 1.5803463668823242, -1.2122983932495117, 0.9614155292510986, -0.3951001763343811, 1.9364351034164429, -0.4929449260234833]

    # encage_rad = [-0.9126984477043152, 0.681510329246521, -2.792526803190927, 1.235825538635254, -0.19241638481616974, 0.31780800223350525, -0.14184236526489258]
    encage_rad = [-1.2732517719268799, 0.7536457777023315, -2.414875030517578, 1.2590689659118652, -0.5527258515357971, 0.1945340484380722, -0.11626409739255905]

    bad_conf = [-62.42737184,  74.01454505, -89.8376216,    3.35964315,   0.,         -86.97984068,  12.49876352]
    bad_conf_rad = [-0.5026999711990356, 1.491589903831482, -1.7011945247650146, -1.052066683769226, 0.1855461299419403, -1.2212766408920288, 0.12732361257076263]

    pose_2_best = [-30.97961758,   37.40778069,  134.93811534,   61.67963412,    0.,  -31.66004362,   62.01843 ]
    pose_2_best_rad = [-0.40043583512306213, 0.7772856950759888, 2.085658550262451, 0.9942722320556641, -0.12471262365579605, -0.6366361975669861, 1.0815138816833496]
    pose_2_encage_rad = [-0.45459818840026855, 0.7141943573951721, 2.4681320190429688, 1.3185760974884033, 0.7203243374824524, 0.8756793141365051, 0.3490450084209442]

    pose_3_best = [-34.19822138,  72.93038487,  81.02961556,  58.14551204,   0.,         -24.79185467, 67.12627577]
    pose_3_best_rad = [-0.6086776256561279, 1.274816870689392, 1.4190993309020996, 0.8291442394256592, -0.08133778721094131, -0.8598849177360535, 1.163482904434204]
    pose_3_encage_rad = [-0.6820478439331055, 1.7339460849761963, 0.8881847858428955, 1.2123607397079468, 1.2596123218536377, 0.14461660385131836, 0.749417245388031]

########
    hammer_pose1_1 = [15.15991239,   31.309434,    136.6298574,    63.95632241,    0.,          -30.0433918, 43.09550091]  # 32  TSoS
    hammer_pose1_2 = [-39.97277693,   68.49677796,  -92.03096497,   36.07265871,    0., -62.10743857,  108.37173285]

    hammer_pose1_3 = [32.58372063,   43.60524663,  104.57057531,   67.40703322,    0., -18.94630362,   37.43938981] # 40 TSoS 
 
    hammer_pose2_1 = [-44.68131595,   49.47312013,  126.85416182,   38.51041846,    0., -0.47768479,  102.7030854] # 41 TSoFS
    hammer_pose2_2 = [-34.69334195,   84.11500842,   54.05829175,   50.91693717,    0., -23.7298625,  127.89131195] #  52 TSoS

    hammer_pose3_1 = [-63.40429812,   68.05819563,  -15.72239283,   34.42383539,    0.,  -43.2590592, -137.31458415] # 45 TSoFS
    hammer_pose3_2 = [-58.77655569,   70.70219125,  -11.84601761,   36.72466369,    0.,  -31.12773962, -118.81631367] #  48 PoFS



    #Hammering New
    hammer_pose3_1 = [-37.99816163,   85.56045267,    9.70945791,   57.6068858,     0.,  -35.51869655, -147.53261364] # 81 PofS
    hammer_pose3_2 = [-33.3056716,    91.58701519,   11.7699677,    78.78093878,    0., -11.71510057,  107.81950385] # 98 TSoS

    hammer_pose2_1 = [-12.71586703,   57.49584048,   74.456615,     64.21642868,    0., -23.05953893,  121.27001287] # 104 TSoFS
    hammer_pose2_2 = [-14.08766612,   51.92725251,   85.73911717,   55.66389844,    0., -35.48729069,  140.58727786] # 86 PoFS
    hammer_pose2_3 = [-31.06544277,   93.58841337,   40.51735399,   64.30377274,   0., -40.00437272,  149.7525938 ] # 101 TSoS

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
    # chosen_primitive = primitives[2]




    #prim 2 position
    deg2rad = pi/180
    some_robot_position_prim2 = [88.7*deg2rad,-39.2*deg2rad,-112.67*deg2rad,-82.9*deg2rad,35*deg2rad,2.05*deg2rad,21.49*deg2rad] # some vertical position


    some_robot_position = [2.75,0.6,-2.9,2.0,2.95,-1.0,-2.9]

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



    # -------------------------
    ## TSOS (old)
    # some_robot_position = mydeg2rad(TSoS_1)
    # some_robot_position = mydeg2rad(TSoS_2)
    # some_robot_position = mydeg2rad(TSoS_3)
    # some_robot_position = mydeg2rad(TSoP)
    # some_robot_position = mydeg2rad(PoFS)
    # some_robot_position = mydeg2rad(TSoFS)

    ##
    # some_robot_position = TSoS_1_rad
    # some_robot_position = TSoS_2_rad
    # some_robot_position = TSoS_3_rad
    # chosen_primitive = tsOs12

    ## TSOP
    # some_robot_position = TSoP_rad
    # chosen_primitive = tsOp

    ## Pofs
    # some_robot_position = PoFS_rad
    # chosen_primitive = pOfs123

    ## TSOFS
    # some_robot_position = TSoFS_rad
    # chosen_primitive = tsOfs123l

    # Encage
    # some_robot_position = encage_rad
    # chosen_primitive = tsOfs123l

    # bad config
    # some_robot_position = mydeg2rad(bad_conf)
    # some_robot_position = bad_conf_rad
    # chosen_primitive = tsOfs123l

    # Pose 3 best
    # some_robot_position = mydeg2rad(pose_3_best)
    # some_robot_position = pose_3_best_rad
    # chosen_primitive = tsOfs123l

    # Pose 3 encage
    # some_robot_position = pose_3_encage_rad
    # chosen_primitive = tsOfs123l

    # Pose 2 best
    # some_robot_position = mydeg2rad(pose_2_best)
    # some_robot_position = pose_2_best_rad
    # chosen_primitive = tsOs12

    # Pose 2 encage
    # some_robot_position = mydeg2rad(pose_2_encage)
    # some_robot_position = pose_2_encage_rad
    # chosen_primitive = tsOfs123l

    ############
    ## HAMMER
    ############
    ## Pose 1
    # some_robot_position = mydeg2rad(hammer_pose1_1)
    # some_robot_position = mydeg2rad(hammer_pose1_2)
    # some_robot_position = mydeg2rad(hammer_pose1_3)
    # chosen_primitive = tsOs12
    # chosen_primitive = tsOp

    ## Pose 2
    # some_robot_position = mydeg2rad(hammer_pose2_1)
    # chosen_primitive = tsOfs123l

    # some_robot_position = mydeg2rad(hammer_pose2_2)
    # chosen_primitive = pOfs123

    # some_robot_position = mydeg2rad(hammer_pose2_3)
    # chosen_primitive = tsOs12

    ## Pose 3
    # some_robot_position = mydeg2rad(hammer_pose3_1)
    # chosen_primitive = pOfs123

    # some_robot_position = mydeg2rad(hammer_pose3_2)
    # chosen_primitive = tsOs12


    # chosen_primitive = ttOft123 #0 TIP
    # chosen_primitive = tsOfs123h#1 SURF H
    # chosen_primitive = tsOfs123l#2 SURF L
    # chosen_primitive = pOfs123  #3 PALM 123
    # chosen_primitive = tsOp     #4 PALM T
    # chosen_primitive = tsOs12   #5 SIDE

    # ------------------------- 


    zero_fingers = [0,0,0,0,  0,0,0,0,  0,0,0,0,  0.4,0,0,0,]

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
