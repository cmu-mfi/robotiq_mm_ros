#! /usr/bin/env python

import sys
import rospy

import actionlib

from robotiq_mm_ros.msg import *

def robotiq_client(args):
    client = actionlib.SimpleActionClient(f'/{args[1]}/move_gripper', GripperCommandAction)

    client.wait_for_server()

    goal = GripperCommandGoal(position=0.05, velocity = 0.1, force=100)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    rospy.init_node('robotiq_client')
    myargv = rospy.myargv(argv=sys.argv)
    robotiq_client(myargv)

