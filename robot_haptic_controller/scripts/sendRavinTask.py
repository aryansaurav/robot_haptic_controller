#!/usr/bin/env python

# Sends messages simulating what Ravin should send me as higher level tasks for my robot's controller

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from my_msgs.msg import Mode
from my_msgs.msg import Task
from math import pi
import sys


def talker():

    print 'Number of arguments:', len(sys.argv), 'arguments.'
    print 'Argument List:', str(sys.argv)

    rospy.init_node('talker', anonymous=True)
    talker.pubTask = rospy.Publisher('task', Task, queue_size=10)
    talker.pubClose = rospy.Publisher('close', Empty, queue_size=1)
    talker.pubTighten = rospy.Publisher('tighten', Empty, queue_size=1)

    task_strings_rotation = ["task", "tr"]
    task_strings_hammering = ["task h", "th"]
    close_strings = ["2", "empty", "close", "c"]
    tighten_strings = ["3", "tighten", "tight", "ti"]

    if len(sys.argv) <= 1:
        rospy.loginfo("Not enough arguments, exiting")
        exit()

    if sys.argv[1] in task_strings_rotation:
        rospy.loginfo("** TASK: rotation **")

        # Send the task 2 times
        sendTask_rotation()

    if sys.argv[1] in task_strings_hammering:
        rospy.loginfo("** TASK: hammering **")

        # Send the task 2 times
        sendTask_hammer()

    elif sys.argv[1] in close_strings:
        rospy.loginfo("** CLOSE **")

        # Send the empty signal
        sendCloseSignal()

    elif sys.argv[1] in tighten_strings:
        rospy.loginfo("** TIGHTEN **")

        # Send the empty signal
        sendTightenSignal()


def sendTask_rotation():
    for i in range(0, 10):
        task = Task()
        task.task_type.data = "rotation"

        task.direction.x = 0.0
        task.direction.y = 0.0
        task.direction.z = 1.0

        task.position.x = 0.0
        task.position.y = 0.0
        task.position.z = 0.0
        task.position.x = -0.36
        task.position.y = 0.61
        task.position.z = 0.0

        task.velocity.data = 5.0 * pi / 180.0

        rospy.loginfo("Sending task")

        talker.pubTask.publish(task)

        time.sleep(0.1)


def sendTask_hammer():
    for i in range(0, 10):
        task = Task()
        task.task_type.data = "hammering"

        task.direction.x = 0.0
        task.direction.y = 0.0
        task.direction.z = 1.0

        # task.direction.x = 0.0
        # task.direction.y = 1.0
        # task.direction.z = 0.0

        task.position.x = 0.0
        task.position.y = 0.0
        task.position.z = 0.0
        task.position.x = 0.0
        task.position.y = 0.0
        task.position.z = 0.0

        task.velocity.data = 5.0 * pi / 180.0

        rospy.loginfo("Sending task")

        talker.pubTask.publish(task)

        time.sleep(0.1)


def sendCloseSignal():
    for i in range(0, 2):

        rospy.loginfo("Sending close signal")
        talker.pubClose.publish(Empty())
        time.sleep(0.3)


def sendTightenSignal():
    for i in range(0, 2):

        rospy.loginfo("Sending tighten signal")
        talker.pubTighten.publish(Empty())
        time.sleep(0.3)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
