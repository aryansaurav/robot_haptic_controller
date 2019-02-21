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

# Simple talker demo that published std_msgs/Strings messages
# to the 'chatter' topic

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from my_msgs.msg import Mode
from std_msgs.msg import Empty


class my_talker(object):

    """A class to send data to my robot_haptic_controller program.
    It receives strings from the voice recognition or simulated using rospub"""

    def __init__(self):
        super(my_talker, self).__init__()  # What does that mean ??

        # Publisher (these notations with "self.*" are wrong, but I'll correct that another time)
        self.pub2 = rospy.Publisher('trajectory', JointState, queue_size=10)
        self.pubMode = rospy.Publisher('mode', Mode, queue_size=10)
        self.pubClose = rospy.Publisher('close', Empty, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        # Subscriber
        rospy.Subscriber("/nl_command_parsed", String, self.callback)

        self.some_robot_position = [2.75, 0.6, -2.9, 2.0, 2.95, -1.0, -2.9]

        # self.Primitives definition
        # OK
        pOfs123 = dict()
        pOfs123['position'] = [0,   0.872664626,   0.872664626,   0.785398163,   0,   0.872664626,  0.872664626,
                               0.785398163,   0,  0.872664626,   0.872664626,   0.785398163,  0.279252680,   0,   0,  0]
        pOfs123['patch1'] = [0.,  0.,  0.,  0.,     0.,  0.,  0.,  0.,     0.,  0.,  0.,  0.,
                             1.1,  0.,  0.,  0.,     0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  1.]
        pOfs123['patch2'] = [0.,  0.7,  1.,  1.,  0.,  0.7,  1.,  1.1,  0.,  0.7,  1., 1.,
                             0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]

        # OK (pb with this one)
        tsOfs123h = dict()
        # tsOfs123h['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                                 # 0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.37881011,
                                 # -0.26179939, -0.17453293,  0.17453293]
        # Changed thumb base position otherwise bug (from the hand ???)
        tsOfs123h['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                                 0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.57881011, 0.06179939, 0.1453293,  0.17453293]
        # tsOfs123h['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                                 # 0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.57881011, 0.06179939, -0.17453293,  0.17453293]
        # Disable thumb tip
        # tsOfs123h['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                               # 0., 0.,  1.1,  0,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
        tsOfs123h['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                               0., 0.,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
        # This patch2 has first phalanxes disabled
        # tsOfs123h['patch2'] = [0.,  0.7,  1.,  1.,  0.,  0.7,  1.,  1.1,  0.,  0.7,  1., 1.,
                               # 0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
        # Removed base of finger contact (hard to reach something with it ...)
        tsOfs123h['patch2'] = [0.,  0.0,  1.,  1.,  0.,  0.0,  1.,  1.1,  0.,  0.0,  1., 1.,
                               0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]


        ###


        my_full_grasp_no_index = dict()

        my_full_grasp_no_index['position'] = [0.,  0.0,  0.0,  0.0,
                                                 0.,  0.83775804,  0.34906585, 0.12217305,
                                                 0.,  0.83775804,  0.34906585,  0.12217305,
                                                 1.67881011, 0.06179939, -0.17453293,  0.17453293]
        my_full_grasp_no_index['patch1']  =   [0.,  0.,  0.,  0.,
                                               0.,  0.,  0.,  0.,
                                               0.,  0.,  0.,  0.,
                                               0.,  0.,  1.,  1.,

                                               0.,  0.,  0.,  0.,
                                               0.,  0.,  0.,  0.,
                                               0.,  0.,  0.,  0.,  0.]

        my_full_grasp_no_index['patch2'] =    [0.,  0.,  0.,  0.0,
                                               0.,  0.,  1.,  1.1,
                                               0.,  0.,  1.,  1.1,
                                               0.,  0.,  0.,  0.,

                                               0.,  0.,  0.,  0.,
                                               0.,  0.,  0.,  0.,
                                               0.,  0.,  0.,  0.,  0.]

        ####
        # OK
        tsOfs123l = dict()
        tsOfs123l['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                                 0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
        tsOfs123l['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                               0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
        tsOfs123l['patch2'] = [0.,  1.,  1.,  0.,  0.,  1.1,  1.,  0.,  0.,  1.,  1.,  0.,
                               0., 0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]

        # OK
        tsOp = dict()
        tsOp['position'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                            0.,  0.,  0.,  1.37881011, -0.1981317, -0.17453293,  1.04719755]
        tsOp['patch1'] = [0.,  0.,  0.,  0.,      0.,  0.,  0.,  0.,      0.,  0.,  0., 0.,
                          0.,  0.7,  1.,  1.1,      0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
        tsOp['patch2'] = [0.,  0.,  0.,  0.,          0.,  0.,  0.,  0.,          0.,  0.,  0.,  0.,
                          0., 0.,  0.,  0.,       0., 0.,  0.0,  0.0,     0.,  0.,  0.,  0.,     1.1,  0., 0.,  0.,     1.]
        # OK
        tsOs12 = dict()
        tsOs12['position'] = [0.30142573,  0.78539816,  1.04719755,  0.78539816,  0.30142573,
                              1.50098316,  0.87266463,  0.29670597,  0.,  0.2,  0.1,  0.1,  0.28, 0.22, -0.09, 0.5]
        tsOs12['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,
                            0.7,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
        tsOs12['patch2'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,
                            0.,  0.,  0.,  0.,  0.7,  1.1,  1.,  0.,  0.7, 1.,  1.,  0.,  0.,  0.,  0.,  0.]
        # OK
        ttOft123 = dict()
        ttOft123['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                                0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
        ttOft123['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                              0., 0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
        ttOft123['patch2'] = [0.,  0.,  0.,  1.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  1.1,
                              0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]

        my_full_grasp = dict()
        my_full_grasp['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,
                                     0.,  0.83775804,  0.34906585, 0.12217305,
                                     0.,  0.83775804,  0.34906585,  0.12217305,
                                     1.37881011, -0.26179939, -0.17453293,  0.17453293]

        finger3 = [0.,  1.,  1.,  1.1,
                   0.,  1.,  1.,  1.1,
                   0.,  1.,  1.,  1.1,
                   0.,  0.,  0.,  0.,

                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,  0.]
        finger3nopink = [0.,  1.,  1.,  1.1,
                   0.,  1.,  1.,  1.1,
                   0.,  0.,  0.,  0.0,
                   0.,  0.,  0.,  0.,

                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,  0.]

        finger3index = [0.,  1.,  1.,  1.1,
                   0.,  0.,  1.,  1.1,
                   0.,  0.,  1.,  1.1,
                   0.,  0.,  0.,  0.,

                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,  0.]

        finger2 = [0.,  0.,  1.,  1.1,
                   0.,  0.,  1.,  1.1,
                   0.,  0.,  1.,  1.1,
                   0.,  0.,  0.,  0.,

                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  0.,  0.]


        # same as tsofsH                                
        my_full_grasp['position'] =[0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                                 0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.57881011, 0.06179939, 0.1453293,  0.17453293]

        my_full_grasp['patch1'] = [0.,  0.,  0.,  0.,
                                   0.,  0.,  0.,  0.,
                                   0.,  0.,  0.,  0.,
                                   0.,  0.,  1.,  1.,

                                   0.,  0.,  0.,  0.,
                                   0.,  0.,  0.,  0.,
                                   0.,  0.,  0.,  0.,  0.]
        my_full_grasp['patch2'] = finger3
        # my_full_grasp['patch2'] = finger3nopink
        # my_full_grasp['patch2'] = finger3index
        # my_full_grasp['patch2'] = finger2

        # my_full_grasp['patch2'] = [0.,  1.,  1.,  1.1,
        #                            0.,  0.,  0.,  0,
        #                            0.,  0.,  0.,  0,
        #                            0.,  0.,  0.,  0.,

        #                            0.,  0.,  0.,  0.,
        #                            0.,  0.,  0.,  0.,
        #                            0.,  0.,  0.,  0.,  0.]

        self.primitives = []
        self.primitives.append(ttOft123)   # 0 TIP
        self.primitives.append(tsOfs123h)  # 1 SURF H
        self.primitives.append(tsOfs123l)  # 2 SURF L
        self.primitives.append(pOfs123)    # 3 PALM 123
        self.primitives.append(tsOp)       # 4 PALM T
        self.primitives.append(tsOs12)     # 5 SIDE
        self.primitives.append(my_full_grasp)     # 5 SIDE
        self.primitives.append(my_full_grasp_no_index)       # 6 PALM T (again)

        self.prim_dict = dict()
        self.prim_dict['grasp zero'] = 0
        self.prim_dict['zero'] = 0

        self.prim_dict['grasp one'] = 1
        self.prim_dict['one'] = 1

        self.prim_dict['grasp two'] = 2
        self.prim_dict['grasp three'] = 3
        self.prim_dict['grasp four'] = 4
        self.prim_dict['grasp katr'] = 4
        self.prim_dict['grasp six'] = 6

        self.prim_dict['grasp seven'] = 5
        self.prim_dict['grasp eight'] = 7

        self.close_str = ["close", "go"]
        self.read_str = ["ready", "stop"]
        self.open_str = ["open", "stop stop"]

        self.last_primitive = 0
        self.last_primitive = 6

        print("waiting for a command ...")
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("----------------------------------")
        rospy.loginfo(" # I heard ***%s***", data.data)

        if data.data in self.prim_dict:
            rospy.loginfo("FOUND :)")
            self.last_primitive = self.prim_dict[data.data]
            # self.send_open_position(0.5)
            self.send_primitive(
                self.prim_dict[data.data], b_mode=True, b_send_trajectory=False, b_send_patches=True)

        elif data.data in self.open_str:
            rospy.loginfo("FOUND :)  Sending OPEN position for a few seconds...")
            self.send_open_position(1.0)
            # Also send an false mode
            self.send_primitive(self.last_primitive,  b_mode=False, b_send_trajectory=False, b_send_patches=True)

        elif data.data in self.close_str:
            rospy.loginfo("FOUND :)  Sending CLOSE order")
            print("last primitive: " + str(self.last_primitive))
            # self.send_primitive(self.last_primitive, b_mode=True, b_send_trajectory=False, b_send_patches=True)
            self.send_primitive(self.last_primitive, b_mode=True, b_send_trajectory=False, b_send_patches=False)

            self.send_close_signal()

        elif data.data in self.read_str:
            rospy.loginfo("FOUND :)  Sending READY order")
            self.send_open_position(0.5)
            # self.send_primitive(self.last_primitive, b_mode=False, b_send_trajectory=True, b_send_patches=True)
            # Do allow to close, but wait for signal.
            self.send_primitive(self.last_primitive, b_mode=True, b_send_trajectory=True, b_send_patches=True)
        else:
            rospy.loginfo("NOT FOUND :(")

    def send_open_position(self, wait_time, number_sends=5):

        zero_fingers = [0, -0.1, 0, 0,    0, -0.1, 0, 0,    0, -0.1, 0, 0,    0.4, -0.1, 0, 0, ]
        joint_state_zero = JointState(
            position=self.some_robot_position + zero_fingers)

        for count in range(0, number_sends):
            self.pub2.publish(joint_state_zero)
            time.sleep(0.1)

        time.sleep(wait_time)
        rospy.loginfo(" Finished waiting")

    def send_close_signal(self):

        self.pubClose.publish(Empty())

    def send_primitive(self, i_primitive, b_mode=True, b_send_trajectory=True, b_send_patches=True):
        # Choose the desired primitive
        chosen_primitive = self.primitives[i_primitive]

        for count in reversed(range(0, 5)):

            joint_state = JointState(
                position=self.some_robot_position + chosen_primitive['position'])

            if b_send_trajectory:
                self.pub2.publish(joint_state)
                rospy.loginfo("Sending traj")

            if b_send_patches:
                # Send desired patches
                mode = Mode()
                mode.mode = b_mode

                mode.patch1 = chosen_primitive['patch1']
                mode.patch2 = chosen_primitive['patch2']

                self.pubMode.publish(mode)

                rospy.loginfo("Sending patches: " + str(count))

                rospy.loginfo(" Finished sending")
            time.sleep(0.1)


if __name__ == '__main__':
    try:
        new = my_talker()
        # init()
    except rospy.ROSInterruptException:
        pass
