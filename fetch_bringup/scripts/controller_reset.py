#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState

# Listen to joy messages, when button held, reset controllers
class ControllerResetTeleop:
    ACTION_NAME = "/query_controller_states"

    def __init__(self):
        rospy.loginfo("Connecting to %s..." % self.ACTION_NAME)
        self.client = actionlib.SimpleActionClient(self.ACTION_NAME, QueryControllerStatesAction)
        self.client.wait_for_server()
        rospy.loginfo("Done.")

        self.start = list()
        self.start.append("arm_controller/gravity_compensation")

        self.stop = list()
        self.stop.append("arm_controller/follow_joint_trajectory")
        self.stop.append("arm_with_torso_controller/follow_joint_trajectory")
        self.stop.append("torso_controller/follow_joint_trajectory")
        self.stop.append("head_controller/follow_joint_trajectory")
        self.stop.append("head_controller/point_head")

        self.reset_button = rospy.get_param("~reset_button", 4)  # default button is the up button

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        try:
            if msg.buttons[self.reset_button] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    self.reset()
                    self.pressed_last = None
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("reset_button is out of range")

    def reset(self):
        goal = QueryControllerStatesGoal()
    
        for controller in self.start:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        for controller in self.stop:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            goal.updates.append(state)

        self.client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("controller_reset")
    c = ControllerResetTeleop()
    rospy.spin()
