#! /usr/bin/env python

import rospy

import actionlib

from robotiq_mm_ros.msg import *

def robotiq_client():
    client = actionlib.SimpleActionClient('/yk_builder/move_gripper', GripperCommandAction)

    print('here')
    client.wait_for_server()
    print('done waiting')
    goal = GripperCommandGoal(position=0.05, velocity = 0.1, force=100)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    rospy.init_node('robotiq_client')
    robotiq_client()

