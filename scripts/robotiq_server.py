#! /usr/bin/env python

import rospy

import actionlib

from robotiq_mm_ros.msg import *
from robotiq_mm_ros.srv import *

from robotiq_mm.robotiq_gripper import RobotiqGripper

class RobotiqServer(object):

    def __init__(self):
        self._gripper = RobotiqGripper("/dev/ttyUSB0")
        self._gripper.calibrate(0,50)
        self._action_name = 'move_gripper'
        self._service_name = 'get_gripper_pos'
        # create messages that are used to publish feedback/result
        self._feedback = GripperCommandFeedback()
        self._result = GripperCommandResult()
        self._as = actionlib.SimpleActionServer(self._action_name, GripperCommmandAction, execute_cb=self.gripper_command_cb, auto_start = False)
        self._as.start()
        self._s = rospy.Service(self._service_name, GetGripperPos, self.get_gripper_position)

    def get_gripper_position(self, req):
        current_gripper_position = self._gripper.getPositionmm() * 0.001
        rospy.loginfo('%s: Current gripper position = %f.' % (self._service_name, current_gripper_position))
        return GetGripperPosResponse(current_gripper_position)
      
    def gripper_command_cb(self, goal):
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.position = self._gripper.getPositionmm() * 0.001
        
        # publish info to the console for the user
        rospy.loginfo('%s: Moving gripper to %f with velocity %f and force %f.' % (self._action_name, goal.position, goal.velocity, goal.force))
        
        desired_position = int(goal.position*1000)
        desired_velocity = int(max(goal.velocity - 0.02, 0.0) * 255 / 0.13)
        desired_force = int(max(goal.force - 20) * 255 / 165)

        self._as.publish_feedback(self._feedback)
        success = self._gripper.goTomm(desired_position, desired_velocity, desired_force)
          
        self._result.success = success
        self._result.position = self._gripper.getPositionmm() * 0.001
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('robotiq_mm_server')
    server = RobotiqServer()
    rospy.spin()
